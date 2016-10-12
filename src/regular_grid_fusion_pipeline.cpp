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

using libcgt::core::cameras::Intrinsics;
using libcgt::core::vecmath::inverse;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::SimilarityTransform;

RegularGridFusionPipeline::RegularGridFusionPipeline(
  const RGBDCameraParameters& camera_params,
  const Vector3i& grid_resolution,
  const SimilarityTransform& world_from_grid,
  bool initial_pose_depth,
  const EuclideanTransform& initial_camera_from_world) :
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

  initial_depth_camera_from_world_(initial_pose_depth ?
    initial_camera_from_world :
    camera_params.depth_from_color * initial_camera_from_world
  ),

  icp_(camera_params.depth.resolution, camera_params.depth.intrinsics,
       camera_params.depth.depth_range)
{
}

void RegularGridFusionPipeline::Reset() {
  num_successive_failures_ = 0;
  depth_pose_history_.clear();
  color_pose_history_estimated_from_depth_.clear();
  regular_grid_.Reset();
}

Array2DView<const uint8_t> RegularGridFusionPipeline::GetBoardImage() const {
  return board_image_.readView();
}

void RegularGridFusionPipeline::SetBoardImage(Array2D<uint8_t> board_image) {
  board_image_ = board_image;
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
  camera.setFrustumFromIntrinsics(camera_params_.depth.intrinsics,
                                  camera_params_.depth.resolution,
                                  camera_params_.depth.depth_range.left(),
                                  camera_params_.depth.depth_range.right());

  if (depth_pose_history_.empty()) {
    camera.setCameraFromWorld(initial_depth_camera_from_world_);
  } else {
    camera.setCameraFromWorld(depth_pose_history_.back().camera_from_world);
  }
  return camera;
}

void RegularGridFusionPipeline::NotifyInputUpdated(bool color_updated,
                                                   bool depth_updated) {
  if (depth_updated) {
    depth_meters_.copyFromHost(input_buffer_.depth_meters);

    depth_processor_.Smooth(depth_meters_, smoothed_depth_meters_);
    depth_processor_.EstimateNormals(smoothed_depth_meters_,
                                     incoming_camera_normals_);
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

  // TODO: Have icp_result write itself into a
  // DeviceArray2D<T>.
  ProjectivePointPlaneICP::Result icp_result =
    icp_.EstimatePose(
      smoothed_depth_meters_, incoming_camera_normals_,
      inverse(depth_pose_history_.back().camera_from_world),
      world_points_, world_normals_,
      pose_estimation_vis_
    );

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

// TODO: use distortion model.
void RegularGridFusionPipeline::Fuse() {
  regular_grid_.Fuse(
    depth_intrinsics_flpp_, camera_params_.depth.depth_range,
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
RegularGridFusionPipeline::ColorPoseHistoryEstimatedDepth() const {
  return color_pose_history_estimated_from_depth_;
}

const std::vector<PoseFrame>&
RegularGridFusionPipeline::DepthPoseHistory() const {
  return depth_pose_history_;
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

EuclideanTransform RegularGridFusionPipeline::GetColorCameraFromWorld(
  const EuclideanTransform& depth_camera_from_world) const {
  return camera_params_.color_from_depth * depth_camera_from_world;
}
