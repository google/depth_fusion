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

#include "libcgt/core/common/ArrayUtils.h"
#include "libcgt/core/imageproc/ColorMap.h"

using libcgt::core::cameras::Intrinsics;
using libcgt::core::vecmath::inverse;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::SimilarityTransform;

namespace {

// TODO: also specify a board configuration.
const char* kArucoDetectorParamsFilename = "../res/detector_params.yaml";
constexpr bool kDoFusion = true;

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
  PoseFrame::EstimationMethod pose_estimation_method,
  const PoseFrame& initial_pose) :
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

  pose_estimation_method_(pose_estimation_method),
  initial_pose_(initial_pose),

  icp_(camera_params.depth.resolution, camera_params.depth.intrinsics,
       camera_params.depth.depth_range),

  aruco_pose_estimator_(camera_params.color, kArucoDetectorParamsFilename) {
  if (pose_estimation_method_ == PoseFrame::EstimationMethod::DEPTH_ICP) {
    assert(initial_pose.method == PoseFrame::EstimationMethod::FIXED_INITIAL);
  } else {
    assert(initial_pose.method == PoseFrame::EstimationMethod::NONE);
  }
}

RegularGridFusionPipeline::RegularGridFusionPipeline(
    const RGBDCameraParameters& camera_params,
    const Vector3i& grid_resolution,
    const SimilarityTransform& world_from_grid,
    const std::vector<PoseFrame>& pose_history) :
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

  icp_(camera_params.depth.resolution, camera_params.depth.intrinsics,
       camera_params.depth.depth_range),

  aruco_pose_estimator_(camera_params.color, kArucoDetectorParamsFilename),

  precomputed_pose_history_(pose_history) {
}

void RegularGridFusionPipeline::Reset() {
  num_successive_failures_ = 0;
  last_raycast_pose_ = {};
  pose_history_.clear();
  regular_grid_.Reset();
}

const RGBDCameraParameters&
RegularGridFusionPipeline::GetCameraParameters() const {
  return camera_params_;
}

PerspectiveCamera RegularGridFusionPipeline::ColorCamera() const {
  PerspectiveCamera camera;
  camera.setFrustum(camera_params_.color.intrinsics,
                    camera_params_.color.resolution,
                    camera_params_.color.depth_range.left(),
                    camera_params_.color.depth_range.right());
  const PoseFrame& pose = pose_history_.empty() ?
    initial_pose_ : pose_history_.back();
  camera.setCameraFromWorld(pose.color_camera_from_world);

  return camera;
}

PerspectiveCamera RegularGridFusionPipeline::DepthCamera() const {
  PerspectiveCamera camera;
  camera.setFrustum(camera_params_.depth.intrinsics,
                    camera_params_.depth.resolution,
                    camera_params_.depth.depth_range.left(),
                    camera_params_.depth.depth_range.right());
  const PoseFrame& pose = pose_history_.empty() ?
    initial_pose_ : pose_history_.back();
  camera.setCameraFromWorld(pose.depth_camera_from_world);

  return camera;
}

void RegularGridFusionPipeline::NotifyInputUpdated(bool color_updated,
                                                   bool depth_updated) {
  assert (color_updated || depth_updated);

  PipelineDataType data_changed = PipelineDataType::NONE;

  bool color_pose_updated = false;
  bool depth_pose_updated = false;

  if (color_updated) {
    data_changed |= PipelineDataType::INPUT_COLOR;
  }
  if (depth_updated) {
    copy(input_buffer_.depth_meters.readView(), depth_meters_);
    depth_processor_.Smooth(depth_meters_, smoothed_depth_meters_);
    depth_processor_.EstimateNormals(smoothed_depth_meters_,
                                     incoming_camera_normals_);
    data_changed |= PipelineDataType::INPUT_DEPTH;
    data_changed |= PipelineDataType::SMOOTHED_DEPTH;
  }

  // TODO: yuck, refactor this.
  if (precomputed_pose_history_.size() > 0) {
    if (color_updated && depth_updated) {
      if (input_buffer_.color_timestamp_ns <
          input_buffer_.depth_timestamp_ns) {
        auto color_itr = FindPoseFrame(precomputed_pose_history_,
          input_buffer_.color_timestamp_ns);
        if (color_itr != std::end(precomputed_pose_history_)) {
          color_pose_updated = true;
          pose_history_.push_back(*color_itr);
        }

        auto depth_itr = FindPoseFrame(precomputed_pose_history_,
          input_buffer_.depth_timestamp_ns);
        if (depth_itr != std::end(precomputed_pose_history_)) {
          depth_pose_updated = true;
          pose_history_.push_back(*depth_itr);
        }
      } else {
        auto depth_itr = FindPoseFrame(precomputed_pose_history_,
          input_buffer_.depth_timestamp_ns);
        if (depth_itr != std::end(precomputed_pose_history_)) {
          depth_pose_updated = true;
          pose_history_.push_back(*depth_itr);
        }

        auto color_itr = FindPoseFrame(precomputed_pose_history_,
          input_buffer_.color_timestamp_ns);
        if (color_itr != std::end(precomputed_pose_history_)) {
          color_pose_updated = true;
          pose_history_.push_back(*color_itr);
        }
      }
    } else if (color_updated) {
      auto color_itr = FindPoseFrame(precomputed_pose_history_,
          input_buffer_.color_timestamp_ns);
      if (color_itr != std::end(precomputed_pose_history_)) {
        color_pose_updated = true;
        pose_history_.push_back(*color_itr);
      }
    } else if (depth_updated) {
      auto depth_itr = FindPoseFrame(precomputed_pose_history_,
          input_buffer_.depth_timestamp_ns);
      if (depth_itr != std::end(precomputed_pose_history_)) {
        depth_pose_updated = true;
        pose_history_.push_back(*depth_itr);
      }
    }
  } else {
    if (pose_estimation_method_ ==
      PoseFrame::EstimationMethod::COLOR_ARUCO) {
      if (color_updated) {
        color_pose_updated = UpdatePoseWithColorCamera();
      }
    } else if (pose_estimation_method_ ==
      PoseFrame::EstimationMethod::DEPTH_ICP) {
      if (depth_updated) {
        depth_pose_updated = UpdatePoseWithDepthCamera();
      }
    } else if (pose_estimation_method_ ==
      PoseFrame::EstimationMethod::COLOR_ARUCO_AND_DEPTH_ICP) {
      // If both color and depth are updated, run them in order.
      if (color_updated && depth_updated) {
        if (input_buffer_.color_timestamp_ns <
          input_buffer_.depth_timestamp_ns) {
          color_pose_updated = UpdatePoseWithColorCamera();
          depth_pose_updated = UpdatePoseWithDepthCamera();
        } else {
          depth_pose_updated = UpdatePoseWithDepthCamera();
          color_pose_updated = UpdatePoseWithColorCamera();
        }
      } else if (color_updated) {
        color_pose_updated = UpdatePoseWithColorCamera();
      } else if (depth_updated) {
        depth_pose_updated = UpdatePoseWithDepthCamera();
      }
    }
  }

  if (color_pose_updated || depth_pose_updated) {
    data_changed |= PipelineDataType::CAMERA_POSE;
    if (depth_pose_updated && kDoFusion) {
      Fuse();
      Raycast();
      data_changed |= PipelineDataType::TSDF;
      data_changed |= PipelineDataType::RAYCAST_NORMALS;
    }
  }
  if (data_changed != PipelineDataType::NONE) {
    emit dataChanged(data_changed);
  }
}

InputBuffer& RegularGridFusionPipeline::GetInputBuffer() {
  return input_buffer_;
}

Box3f RegularGridFusionPipeline::TSDFGridBoundingBox() const {
  return regular_grid_.BoundingBox();
}

const SimilarityTransform&
RegularGridFusionPipeline::TSDFWorldFromGridTransform() const {
  return regular_grid_.WorldFromGrid();
}


bool RegularGridFusionPipeline::UpdatePoseWithColorCamera()
{
  assert (pose_estimation_method_ ==
    PoseFrame::EstimationMethod::COLOR_ARUCO ||
    pose_estimation_method_ ==
    PoseFrame::EstimationMethod::COLOR_ARUCO_AND_DEPTH_ICP);
  ArucoPoseEstimator::Result result =
    aruco_pose_estimator_.EstimatePose(input_buffer_.color_bgr_ydown);
  if (result.valid)
  {
    PoseFrame pose_frame;
    pose_frame.method = PoseFrame::EstimationMethod::COLOR_ARUCO;
    pose_frame.frame_index = input_buffer_.color_frame_index;
    pose_frame.timestamp_ns = input_buffer_.color_timestamp_ns;
    pose_frame.color_camera_from_world = inverse(result.world_from_camera);
    pose_frame.depth_camera_from_world =
      camera_params_.ConvertToDepthCameraFromWorld(
        pose_frame.color_camera_from_world);

    pose_history_.push_back(pose_frame);
  }

  return result.valid;
}

bool RegularGridFusionPipeline::UpdatePoseWithDepthCamera() {
  assert (pose_estimation_method_ ==
    PoseFrame::EstimationMethod::DEPTH_ICP ||
    pose_estimation_method_ ==
    PoseFrame::EstimationMethod::COLOR_ARUCO_AND_DEPTH_ICP);

  // ICP can only estimate relative pose, not absolute. I.e., it needs a
  // previous pose before it can estimate the current pose.
  //
  // In ICP-only mode, initialize with a user-provide default pose.
  // In fiducial + ICP mode, do nothing until we estimated a pose from the
  // fiducial.
  if (pose_history_.empty()) {
    if (pose_estimation_method_ == PoseFrame::EstimationMethod::DEPTH_ICP) {
      pose_history_.push_back(initial_pose_);
      return true;
    } else if (pose_estimation_method_ ==
      PoseFrame::EstimationMethod::COLOR_ARUCO_AND_DEPTH_ICP ) {
      return false;
    }
  }

  // Initialization: in COLOR+DEPTH_ICP mode, we wait until we've found the
  // fiducial marker in the color frame before fusing and raycasting. But when
  // we process the first depth frame, we will not have a previous depth pose
  // yet. In this case, return true: it will use the last color frame.
  if (last_raycast_pose_.method == PoseFrame::EstimationMethod::NONE) {
    return true;
  }

  // In COLOR+DEPTH_ICP mode, if there's been a more recent pose estimate than
  // the last raycast, update the raycast.
  if (last_raycast_pose_.timestamp_ns < pose_history_.back().timestamp_ns) {
    Raycast();
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
    pose_frame.method = PoseFrame::EstimationMethod::DEPTH_ICP;
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
  regular_grid_.Raycast(
    depth_intrinsics_flpp_,
    inverse(last_raycast_pose_.depth_camera_from_world).asMatrix(),
    world_points_, world_normals_
  );
}

void RegularGridFusionPipeline::Raycast(const PerspectiveCamera& camera,
                                        DeviceArray2D<float4>& world_points,
                                        DeviceArray2D<float4>& world_normals) {
  Intrinsics intrinsics = camera.intrinsics(world_points.size());
  Vector4f flpp{intrinsics.focalLength, intrinsics.principalPoint};

  regular_grid_.Raycast(
    flpp,
    camera.worldFromCamera().asMatrix(),
    world_points, world_normals
  );
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
