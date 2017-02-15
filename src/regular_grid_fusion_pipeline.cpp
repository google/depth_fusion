// Copyright 2016 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http ://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "regular_grid_fusion_pipeline.h"

#include <cassert>

#include <gflags/gflags.h>

#include "libcgt/core/common/ArrayUtils.h"
#include "libcgt/core/imageproc/ColorMap.h"

using libcgt::core::arrayutils::flipYInPlace;
using libcgt::core::cameras::Intrinsics;
using libcgt::core::vecmath::inverse;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::SimilarityTransform;

DECLARE_bool(adaptive_raycast);

namespace {

// TODO: also specify a board configuration.
const char* kArucoDetectorParamsFilename = "../res/detector_params.yaml";
constexpr int kSingleMarkerFiducialId = 3;
constexpr bool kDoFusion = true;

std::vector<PoseFrame>::const_iterator FindPoseFrameByFrameIndex(
  const std::vector<PoseFrame>& pose_history,
  int32_t frame_index) {
  return std::find_if(std::begin(pose_history), std::end(pose_history),
    [&](const PoseFrame& f) {
    return f.frame_index == frame_index;
  });
}

std::vector<PoseFrame>::const_iterator FindPoseFrame(
  const std::vector<PoseFrame>& pose_history,
  int64_t timestamp_ns) {
  return std::find_if(std::begin(pose_history), std::end(pose_history),
    [&](const PoseFrame& f) {
    return f.timestamp_ns == timestamp_ns;
  });
}

}

RegularGridFusionPipeline::RegularGridFusionPipeline(
  const RGBDCameraParameters& camera_params,
  const Vector3i& grid_resolution,
  const SimilarityTransform& world_from_grid,
  const PoseEstimatorOptions& pose_estimator_options) :
  depth_meters_(camera_params.depth.resolution),
  smoothed_depth_meters_(camera_params.depth.resolution),
  incoming_camera_normals_(camera_params.depth.resolution),
  world_points_(camera_params.depth.resolution),
  world_normals_(camera_params.depth.resolution),
  pose_estimation_vis_(camera_params.depth.resolution),

  input_buffer_(camera_params.color.resolution,
                camera_params.depth.resolution),

  regular_grid_(grid_resolution, world_from_grid),

  camera_params_(camera_params),
  depth_intrinsics_flpp_{
    camera_params.depth.intrinsics.focalLength,
    camera_params.depth.intrinsics.principalPoint
  },
  depth_range_(camera_params.depth.depth_range),

  depth_processor_(camera_params.depth.intrinsics,
    camera_params.depth.depth_range),

  pose_estimator_options_(pose_estimator_options),

  icp_(camera_params.depth.resolution, camera_params.depth.intrinsics,
       camera_params.depth.depth_range),

  //aruco_pose_estimator_(aruco_cube_fiducial_, camera_params.color,
  aruco_single_marker_fiducial_(SingleMarkerFiducial::kDefaultSideLength,
    kSingleMarkerFiducialId),
  aruco_pose_estimator_(aruco_single_marker_fiducial_, camera_params.color,
    kArucoDetectorParamsFilename),
  aruco_vis_(camera_params.color.resolution) {
  // TODO: CheckPoseEstimatorOptions().
}

void RegularGridFusionPipeline::Reset() {
  num_successive_failures_ = 0;
  last_raycast_pose_ = {};
  pose_history_.clear();
  is_first_depth_frame_ = true;
  regular_grid_.Reset();
}

const RGBDCameraParameters&
RegularGridFusionPipeline::GetCameraParameters() const {
  return camera_params_;
}

const CubeFiducial& RegularGridFusionPipeline::GetArucoCubeFiducial() const {
  return aruco_cube_fiducial_;
}

PerspectiveCamera RegularGridFusionPipeline::ColorCamera() const {
  PerspectiveCamera camera;
  camera.setFrustum(camera_params_.color.intrinsics,
                    Vector2f(camera_params_.color.resolution),
                    camera_params_.color.depth_range.left(),
                    camera_params_.color.depth_range.right());
  EuclideanTransform pose = pose_history_.empty() ?
    EuclideanTransform() :
    pose_history_.back().color_camera_from_world;
  camera.setCameraFromWorld(pose);
  return camera;
}

PerspectiveCamera RegularGridFusionPipeline::DepthCamera() const {
  PerspectiveCamera camera;
  camera.setFrustum(camera_params_.depth.intrinsics,
                    Vector2f(camera_params_.depth.resolution),
                    camera_params_.depth.depth_range.left(),
                    camera_params_.depth.depth_range.right());
  EuclideanTransform pose = pose_history_.empty() ?
    EuclideanTransform() :
    pose_history_.back().depth_camera_from_world;
  camera.setCameraFromWorld(pose);
  return camera;
}

void RegularGridFusionPipeline::NotifyColorUpdated() {
  PipelineDataType data_changed = PipelineDataType::INPUT_COLOR;

  bool pose_updated = false;

  if (pose_estimator_options_.method == PoseEstimationMethod::PRECOMPUTED ||
    pose_estimator_options_.method ==
    PoseEstimationMethod::PRECOMPUTED_REFINE_WITH_DEPTH_ICP) {
    auto color_itr = FindPoseFrame(pose_estimator_options_.precomputed_path,
      input_buffer_.color_timestamp_ns);
    if (color_itr != pose_estimator_options_.precomputed_path.end()) {
      data_changed |= PipelineDataType::CAMERA_POSE;
      pose_history_.push_back(*color_itr);
      pose_updated = true;
    }
  } else if (
    pose_estimator_options_.method == PoseEstimationMethod::COLOR_ARUCO ||
    pose_estimator_options_.method ==
      PoseEstimationMethod::COLOR_ARUCO_AND_DEPTH_ICP) {
    if (UpdatePoseWithColorCamera()) {
      data_changed |= PipelineDataType::CAMERA_POSE;
      pose_updated = true;
    }
    data_changed |= PipelineDataType::POSE_ESTIMATION_VIS;
  }

  if (pose_updated) {
    Raycast();
  }

  if (data_changed != PipelineDataType::NONE) {
    emit dataChanged(data_changed);
  }
}

void RegularGridFusionPipeline::NotifyDepthUpdated() {
  // TODO: protect visualization buffers with a mutex
  PipelineDataType data_changed = PipelineDataType::INPUT_DEPTH;

  copy(input_buffer_.depth_meters.readView(), depth_meters_);
  depth_processor_.Smooth(depth_meters_, smoothed_depth_meters_);
  depth_processor_.EstimateNormals(smoothed_depth_meters_,
                                    incoming_camera_normals_);
  data_changed |= PipelineDataType::SMOOTHED_DEPTH;

  bool pose_updated = false;
  PoseEstimationMethod method = pose_estimator_options_.method;

  // TODO: PRECOMPUTED_REFINE_WITH_DEPTH_ICP
  if (method == PoseEstimationMethod::PRECOMPUTED) {
    auto itr = FindPoseFrame(pose_estimator_options_.precomputed_path,
      input_buffer_.depth_timestamp_ns);
    if (itr != pose_estimator_options_.precomputed_path.end()) {
      data_changed |= PipelineDataType::CAMERA_POSE;
      pose_history_.push_back(*itr);
      pose_updated = true;
    }
  } else if (method == PoseEstimationMethod::DEPTH_ICP) {
    // In DEPTH_ICP mode, if it's the very first depth frame, use the provided
    // initial pose.
    if (is_first_depth_frame_) {
      is_first_depth_frame_ = false;
      pose_updated = true;
    } else if (UpdatePoseWithDepthCamera()) {
      data_changed |= PipelineDataType::POSE_ESTIMATION_VIS;
      pose_updated = true;
    }
  } else if (method == PoseEstimationMethod::COLOR_ARUCO_AND_DEPTH_ICP) {
    // In COLOR+DEPTH_ICP mode, if we get depth frames without an absolute pose
    // estimate from color tracking, UpdatePoseWithDepthCamera() will fail
    // since nothing will have been raycast. This has the effect of waiting for
    // a color frame to lock on.
    if (UpdatePoseWithDepthCamera()) {
      data_changed |= PipelineDataType::POSE_ESTIMATION_VIS;
      pose_updated = true;
    }
  }

  if (pose_updated && kDoFusion) {
    Fuse();
    Raycast();
    data_changed |= PipelineDataType::TSDF;
    data_changed |= PipelineDataType::RAYCAST_NORMALS;
  }

  if (data_changed != PipelineDataType::NONE) {
    emit dataChanged(data_changed);
  }
}

InputBuffer& RegularGridFusionPipeline::GetInputBuffer() {
  return input_buffer_;
}

Array2DReadView<uint8x3 >
RegularGridFusionPipeline::GetColorPoseEstimatorVisualization() const {
  return aruco_vis_;
}

Box3f RegularGridFusionPipeline::TSDFGridBoundingBox() const {
  return regular_grid_.BoundingBox();
}

const SimilarityTransform&
RegularGridFusionPipeline::TSDFWorldFromGridTransform() const {
  return regular_grid_.WorldFromGrid();
}

// TODO: make this a pure function and have it take as parameters the last
// raycast pose, last raycast image, and incoming image.
// The caller can push it onto history.
bool RegularGridFusionPipeline::UpdatePoseWithColorCamera()
{
  assert (pose_estimator_options_.method ==
    PoseEstimationMethod::COLOR_ARUCO ||
    pose_estimator_options_.method ==
    PoseEstimationMethod::COLOR_ARUCO_AND_DEPTH_ICP);
  ArucoPoseEstimator::Result result =
    aruco_pose_estimator_.EstimatePose(input_buffer_.color_bgr_ydown,
      aruco_vis_);

  if (result.valid) {
    PoseFrame pose_frame;
    pose_frame.frame_index = input_buffer_.color_frame_index;
    pose_frame.timestamp_ns = input_buffer_.color_timestamp_ns;
    pose_frame.color_camera_from_world = inverse(result.world_from_camera);
    pose_frame.depth_camera_from_world =
      camera_params_.ConvertToDepthCameraFromWorld(
        pose_frame.color_camera_from_world);

    pose_history_.push_back(pose_frame);
  }

  // Flip visualization upside down.
  flipYInPlace(aruco_vis_.writeView());

  return result.valid;
}

// TODO: make this a pure function and have it take as parameters the last
// raycast pose, last raycast image, and incoming image.
// The caller can push it onto history. The caller would also be the one
// that maintains the number of successive failures.
bool RegularGridFusionPipeline::UpdatePoseWithDepthCamera() {
  // ICP can only estimate relative pose, not absolute. I.e., it needs both a
  // previous pose and a 3D model before it can estimate the current pose.
  if (last_raycast_pose_.timestamp_ns == 0) {
    return false;
  }

  // TODO: Have icp_result write itself into a DeviceArray2D<T>.
  ProjectivePointPlaneICP::Result icp_result = icp_.EstimatePose(
      smoothed_depth_meters_, incoming_camera_normals_,
      inverse(last_raycast_pose_.depth_camera_from_world),
      world_points_, world_normals_,
      pose_estimation_vis_
    );

  if (icp_result.valid) {
    PoseFrame pose_frame;
    pose_frame.timestamp_ns = input_buffer_.depth_timestamp_ns;
    pose_frame.frame_index = input_buffer_.depth_frame_index;
    pose_frame.depth_camera_from_world = inverse(icp_result.world_from_camera);
    pose_frame.color_camera_from_world =
      camera_params_.ConvertToColorCameraFromWorld(
        pose_frame.depth_camera_from_world);

    pose_history_.push_back(pose_frame);
  } else {
    ++num_successive_failures_;
    if (num_successive_failures_ > kMaxSuccessiveFailuresBeforeReset) {
      Reset();
    }
  }
  return icp_result.valid;
}

// TODO: use distortion model.
void RegularGridFusionPipeline::Fuse() {
  regular_grid_.Fuse(
    depth_intrinsics_flpp_, camera_params_.depth.depth_range,
    pose_history_.back().depth_camera_from_world.asMatrix(),
    depth_meters_
  );
}

void RegularGridFusionPipeline::Raycast() {
  last_raycast_pose_ = pose_history_.back();

  if (FLAGS_adaptive_raycast) {
    regular_grid_.AdaptiveRaycast(
      depth_intrinsics_flpp_,
      inverse(last_raycast_pose_.depth_camera_from_world).asMatrix(),
      world_points_, world_normals_
    );
  } else {
    regular_grid_.Raycast(
      depth_intrinsics_flpp_,
      inverse(last_raycast_pose_.depth_camera_from_world).asMatrix(),
      world_points_, world_normals_
    );
  }
}

void RegularGridFusionPipeline::Raycast(const PerspectiveCamera& camera,
                                        DeviceArray2D<float4>& world_points,
                                        DeviceArray2D<float4>& world_normals) {
  Intrinsics intrinsics = camera.intrinsics(Vector2f(world_points.size()));
  Vector4f flpp{intrinsics.focalLength, intrinsics.principalPoint};

  if (FLAGS_adaptive_raycast) {
    regular_grid_.AdaptiveRaycast(
      flpp,
      camera.worldFromCamera().asMatrix(),
      world_points, world_normals
    );
  } else {
    regular_grid_.Raycast(
      flpp,
      camera.worldFromCamera().asMatrix(),
      world_points, world_normals
    );
  }
}

TriangleMesh RegularGridFusionPipeline::Triangulate() const {
  return regular_grid_.Triangulate();
}

const std::vector<PoseFrame>&
RegularGridFusionPipeline::PoseHistory() const {
  return pose_history_;
}

const DeviceArray2D<float>&
RegularGridFusionPipeline::SmoothedDepthMeters() const
{
  return smoothed_depth_meters_;
}

const DeviceArray2D<float4>&
RegularGridFusionPipeline::SmoothedIncomingNormals() const
{
  return incoming_camera_normals_;
}

const DeviceArray2D<uchar4>&
RegularGridFusionPipeline::PoseEstimationVisualization() const {
  return pose_estimation_vis_;
}

const DeviceArray2D<float4>&
RegularGridFusionPipeline::RaycastNormals() const {
  return world_normals_;
}
