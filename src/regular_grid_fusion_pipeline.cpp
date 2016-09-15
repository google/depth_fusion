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

#include <core/common/ArrayUtils.h>
#include <core/imageproc/ColorMap.h>

using libcgt::core::arrayutils::cast;
using libcgt::core::imageproc::linearRemapToLuminance;
using libcgt::core::imageproc::normalsToRGBA;
using libcgt::core::vecmath::inverse;
using libcgt::core::vecmath::EuclideanTransform;

RegularGridFusionPipeline::RegularGridFusionPipeline(
  const RGBDCameraParameters& camera_params,
  const Vector3i& grid_resolution, float voxel_size,
  const SimilarityTransform& world_from_grid,
  bool initial_pose_depth,
  const EuclideanTransform& initial_camera_from_world) :
  depth_meters_(camera_params.depth_resolution),
  smoothed_depth_meters_(camera_params.depth_resolution),
  incoming_camera_normals_(camera_params.depth_resolution),
  world_points_(camera_params.depth_resolution),
  world_normals_(camera_params.depth_resolution),
  icp_debug_vis_(camera_params.depth_resolution),

  input_buffer_(camera_params.color_resolution,
    camera_params.depth_resolution),

  regular_grid_(grid_resolution, voxel_size, world_from_grid),

  camera_params_(camera_params),
  depth_intrinsics_flpp_{
    camera_params.depth_intrinsics.focalLength,
    camera_params.depth_intrinsics.principalPoint
  },
  depth_range_(camera_params.depth_range),

  depth_processor_(camera_params.depth_intrinsics, camera_params.depth_range),

  initial_depth_camera_from_world_(initial_pose_depth ?
    initial_camera_from_world :
    camera_params.depth_from_color * initial_camera_from_world
  ),

  icp_(camera_params.depth_resolution, camera_params.depth_intrinsics,
    camera_params.depth_range),

  debug_smoothed_depth_meters_(camera_params.depth_resolution),
  debug_smoothed_depth_vis_(camera_params.depth_resolution),
  debug_incoming_camera_normals_(camera_params.depth_resolution),
  debug_points_world_(camera_params.depth_resolution),
  debug_normals_world_(camera_params.depth_resolution),
  debug_icp_debug_vis_(camera_params.depth_resolution)
{
}

void RegularGridFusionPipeline::Reset() {
  num_successive_failures_ = 0;
  depth_pose_history_.clear();
  color_pose_history_estimated_from_depth_.clear();
  regular_grid_.Reset();
}

PerspectiveCamera RegularGridFusionPipeline::ColorCamera() const {
  PerspectiveCamera camera;
  camera.setFrustumFromIntrinsics(camera_params_.color_intrinsics,
    camera_params_.color_resolution,
    camera_params_.color_range.left(), camera_params_.color_range.right());

  if (color_pose_history_estimated_from_depth_.empty()) {
    camera.setCameraFromWorld(GetColorCameraFromWorld(
      initial_depth_camera_from_world_));
  } else {
    camera.setCameraFromWorld(GetColorCameraFromWorld(
      depth_pose_history_.back().camera_from_world));
  }
  return camera;
}

PerspectiveCamera RegularGridFusionPipeline::DepthCamera() const {
  PerspectiveCamera camera;
  camera.setFrustumFromIntrinsics(camera_params_.depth_intrinsics,
    camera_params_.depth_resolution,
    camera_params_.depth_range.left(), camera_params_.depth_range.right());

  if (depth_pose_history_.empty()) {
    camera.setCameraFromWorld(initial_depth_camera_from_world_);
  }
  else {
    camera.setCameraFromWorld(depth_pose_history_.back().camera_from_world);
  }
  return camera;
}

void RegularGridFusionPipeline::NotifyInputUpdated(bool color_updated,
  bool depth_updated) {
  if (depth_updated) {
    depth_meters_.copyFromHost(input_buffer_.depth_meters);

    depth_processor_.SmoothDepth(depth_meters_, smoothed_depth_meters_);
    depth_processor_.EstimateNormals(smoothed_depth_meters_,
      incoming_camera_normals_);

    // TODO(jiawen): make these queryable by the visualizer
    smoothed_depth_meters_.copyToHost(debug_smoothed_depth_meters_);
    incoming_camera_normals_.copyToHost(
      cast<float4>(debug_incoming_camera_normals_.writeView()));

    // TODO(jiawen): interop
    linearRemapToLuminance(debug_smoothed_depth_meters_, depth_range_,
      Range1f::fromMinMax(0.2f, 1.0f), debug_smoothed_depth_vis_);
    normalsToRGBA(debug_incoming_camera_normals_,
      debug_incoming_camera_normals_);
  }
}

bool RegularGridFusionPipeline::UpdatePoseWithICP() {
  // On the first frame, initialize history with default.
  if (depth_pose_history_.empty()) {
    PoseFrame depth_pose_frame;
    depth_pose_frame.timestamp = input_buffer_.depth_timestamp;
    depth_pose_frame.frame_index = input_buffer_.depth_frame_index;
    depth_pose_frame.camera_from_world = initial_depth_camera_from_world_;

    depth_pose_history_.push_back(depth_pose_frame);

    PoseFrame color_pose_frame = depth_pose_frame;
    color_pose_frame.camera_from_world = GetColorCameraFromWorld(
      depth_pose_frame.camera_from_world);
    color_pose_history_estimated_from_depth_.push_back(color_pose_frame);

    return true;
  }

  // TODO(jiawen): make this queryable. have icp_result write itself into a
  // DeviceArray2D<T>.
  ProjectivePointPlaneICP::Result icp_result =
    icp_.EstimatePose(
      smoothed_depth_meters_, incoming_camera_normals_,
      inverse(depth_pose_history_.back().camera_from_world),
      world_points_, world_normals_,
      icp_debug_vis_
    );

  // TODO(jiawen): interop
  icp_debug_vis_.copyToHost(cast<uchar4>(debug_icp_debug_vis_.writeView()));

  if (icp_result.valid) {
    PoseFrame depth_pose_frame;
    depth_pose_frame.timestamp = input_buffer_.depth_timestamp;
    depth_pose_frame.frame_index = input_buffer_.depth_frame_index;
    depth_pose_frame.camera_from_world = inverse(icp_result.world_from_camera);
    depth_pose_history_.push_back(depth_pose_frame);

    PoseFrame color_pose_frame = depth_pose_frame;
    color_pose_frame.camera_from_world = GetColorCameraFromWorld(
      depth_pose_frame.camera_from_world);
    color_pose_history_estimated_from_depth_.push_back(color_pose_frame);
  } else {
    ++num_successive_failures_;
    if (num_successive_failures_ > kMaxSuccessiveFailuresBeforeReset) {
      Reset();
    }
  }
  return icp_result.valid;
}

void RegularGridFusionPipeline::Fuse() {
  regular_grid_.Fuse(
    depth_intrinsics_flpp_, camera_params_.depth_range,
    depth_pose_history_.back().camera_from_world.asMatrix(),
    depth_meters_
  );
}

void RegularGridFusionPipeline::Raycast() {
  regular_grid_.Raycast(
    depth_intrinsics_flpp_,
    inverse(depth_pose_history_.back().camera_from_world).asMatrix(),
    world_points_, world_normals_
  );

  // TODO(jiawen): and these!
  world_points_.copyToHost(debug_points_world_.writeView());
  world_normals_.copyToHost(debug_normals_world_.writeView());
}

TriangleMesh RegularGridFusionPipeline::Triangulate() const {
  return regular_grid_.Triangulate();
}

const std::vector<PoseFrame>&
  RegularGridFusionPipeline::ColorPoseHistoryEstimatedDepth() const {
  return color_pose_history_estimated_from_depth_;
}

const std::vector<PoseFrame>&
  RegularGridFusionPipeline::DepthPoseHistory() const {
  return depth_pose_history_;
}

EuclideanTransform RegularGridFusionPipeline::GetColorCameraFromWorld(
  const EuclideanTransform& depth_camera_from_world) const {
  return camera_params_.color_from_depth * depth_camera_from_world;
}
