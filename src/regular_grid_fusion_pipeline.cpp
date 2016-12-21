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

#include <core/common/ArrayUtils.h>
#include <core/imageproc/ColorMap.h>

using libcgt::core::cameras::Intrinsics;
using libcgt::core::vecmath::inverse;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::SimilarityTransform;

RegularGridFusionPipeline::RegularGridFusionPipeline(
  const RGBDCameraParameters& camera_params,
  const Vector3i& grid_resolution,
  const SimilarityTransform& world_from_grid,
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

  initial_pose_(initial_pose),

  icp_(camera_params.depth.resolution, camera_params.depth.intrinsics,
       camera_params.depth.depth_range)
  // TODO: pass in a PoseEstimator object.
#ifdef USE_ARTOOLKIT
  ,color_pose_estimator_(camera_params.color, "../res/cube_64mm.dat")
#endif
{
  assert(initial_pose.method == PoseFrame::EstimationMethod::FIXED_INITIAL);
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
  camera.setFrustumFromIntrinsics(camera_params_.color.intrinsics,
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
  camera.setFrustumFromIntrinsics(camera_params_.depth.intrinsics,
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
  PipelineDataType data_changed = PipelineDataType::NONE;

  bool pose_updated = false;
  if (color_updated) {
    // TODO: check if timestamps are close?
    // int64_t color_timestamp_ns = input_buffer_.color_timestamp_ns;
    bool color_pose_updated = UpdatePoseWithColorCamera();
    pose_updated |= color_pose_updated;
    data_changed |= PipelineDataType::INPUT_COLOR;
  }
  if (depth_updated) {
    depth_meters_.copyFromHost(input_buffer_.depth_meters);
    depth_processor_.Smooth(depth_meters_, smoothed_depth_meters_);
    depth_processor_.EstimateNormals(smoothed_depth_meters_,
                                     incoming_camera_normals_);
    data_changed |= PipelineDataType::INPUT_DEPTH;
    data_changed |= PipelineDataType::SMOOTHED_DEPTH;

    bool depth_pose_updated = UpdatePoseWithICP();
    pose_updated |= depth_pose_updated;

    if (pose_updated) {
#if 1
      // TODO: if do_fusion...
      Fuse();
      Raycast();
      data_changed |= PipelineDataType::TSDF;
      data_changed |= PipelineDataType::RAYCAST_NORMALS;
#endif
    }
  }

  if (pose_updated) {
    data_changed |= PipelineDataType::CAMERA_POSE;
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

bool RegularGridFusionPipeline::UpdatePoseWithICP() {
  // ICP can only estimate relative pose, not absolute (it needs a previous
  // raycast). To get started, if there wasn't a frame estimated using the
  // color camera, initialize pose history with user-specified default.
  if (pose_history_.empty()) {
    pose_history_.push_back(initial_pose_);
    return true;
  }

  // TODO: this is a lot of combinations, might be easier to simplify this
  // logic by limiting the number of configurations.
  // RGBD mode: don't start until we saw a fiducial first
  // RGB tracking only mode: easy
  // Depth tracking only mode: easy

  // If we have never raycast before but have a pose (say, from color
  // tracking), then make sure to do a raycast first.
  if (last_raycast_pose_.method == PoseFrame::EstimationMethod::NONE) {
    Raycast();
  }

  // TODO: Have icp_result write itself into a
  // DeviceArray2D<T>.
  ProjectivePointPlaneICP::Result icp_result =
    icp_.EstimatePose(
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

bool RegularGridFusionPipeline::UpdatePoseWithColorCamera() {
#ifdef USE_ARTOOLKIT
  ARToolkitPoseEstimator::Result result =
    color_pose_estimator_.EstimatePose(input_buffer_.color_bgr_ydown);
  if (result.valid) {
    PoseFrame pose_frame;
    pose_frame.method = PoseFrame::EstimationMethod::COLOR_ARTOOLKIT;
    pose_frame.frame_index = input_buffer_.color_frame_index;
    pose_frame.timestamp_ns = input_buffer_.color_timestamp_ns;
    pose_frame.color_camera_from_world = inverse(result.world_from_camera);
    pose_frame.depth_camera_from_world =
      camera_params_.ConvertToDepthCameraFromWorld(
        pose_frame.color_camera_from_world);

    pose_history_.push_back(pose_frame);
  }

  return result.valid;
#else
  return false;
#endif
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
