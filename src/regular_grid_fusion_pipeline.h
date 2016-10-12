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
#ifndef REGULAR_GRID_FUSION_PIPELINE_H
#define REGULAR_GRID_FUSION_PIPELINE_H

#include <core/cameras/PerspectiveCamera.h>
#include <core/geometry/TriangleMesh.h>
#include <core/vecmath/Vector2i.h>
#include <core/vecmath/Vector3i.h>
#include <core/vecmath/EuclideanTransform.h>
#include <cuda/DeviceArray2D.h>

#include "regular_grid_tsdf.h"
#include "rgbd_camera_parameters.h"
#include "depth_processor.h"
#include "projective_point_plane_icp.h"
#include "input_buffer.h"
#include "pose_frame.h"

class RegularGridFusionPipeline {

  using EuclideanTransform = libcgt::core::vecmath::EuclideanTransform;
  using SimilarityTransform = libcgt::core::vecmath::SimilarityTransform;

 public:

  // TODO(jiawen): Allow initial pose using color from world or
  // depth from world but not both.
  //
  RegularGridFusionPipeline(
    const RGBDCameraParameters& camera_params,
    const Vector3i& grid_resolution,
    const SimilarityTransform& world_from_grid,
    // true if it's the depth camera, false if it's the color camera
    bool initial_pose_depth,
    const EuclideanTransform& initial_camera_from_world);

  Array2DView<const uint8_t> GetBoardImage() const;
  void SetBoardImage(Array2D<uint8_t> board_image);

  void Reset();

  const RGBDCameraParameters& GetCameraParameters() const;

  void NotifyInputUpdated(bool color_updated, bool depth_updated);

  InputBuffer& GetInputBuffer();

  // TODO: oriented box class
  Box3f TSDFGridBoundingBox() const;
  const SimilarityTransform& TSDFWorldFromGridTransform() const;

  PerspectiveCamera ColorCamera() const;
  PerspectiveCamera DepthCamera() const;

  // Use icp to estimate the
  // TODO(jiawen): consider writing as a single function that:
  // - computes pose of the incoming frame
  // - fuses it
  // - runs raycasting for display and caches it for the next frame
  bool UpdatePoseWithICP();

  // Update the regular grid with the latest image.
  void Fuse();

  // Update world_points_ and world_normals_ by raycasting the latest regular
  // grid from the current pose.
  void Raycast();

  void Raycast(const PerspectiveCamera& camera,
               DeviceArray2D<float4>& world_points,
               DeviceArray2D<float4>& world_normals);

  TriangleMesh Triangulate() const;

  // Returns CameraFromworld.
  // Each frame is directly transformed from those in DepthPoseHistory.
  // I.e., each frame has the same frame_index and timestamp, and the transform
  // is simply color_from_depth * DepthPoseHistory[i].camera_from_world.
  const std::vector<PoseFrame>& ColorPoseHistoryEstimatedDepth() const;

  // Return CameraFromWorld.
  const std::vector<PoseFrame>& DepthPoseHistory() const;

  const DeviceArray2D<float>& SmoothedDepthMeters() const;

  // In camera space.
  const DeviceArray2D<float4>& SmoothedIncomingNormals() const;

  const DeviceArray2D<uchar4>& PoseEstimationVisualization() const;

  // In world space.
  const DeviceArray2D<float4>& RaycastNormals() const;

 private:
  // CPU input buffers.
  InputBuffer input_buffer_;

  // ----- Input copied to the GPU -----
  // Incoming depth frame in meters.
  DeviceArray2D<float> depth_meters_;

  // ----- Pipeline intermediates -----

  // Incoming depth smoothed using a bilateral filter.
  DeviceArray2D<float> smoothed_depth_meters_;
  // Incoming camera-space normals, estimated from smoothed depth.
  DeviceArray2D<float4> incoming_camera_normals_;

  // Pose estimation visualization.
  DeviceArray2D<uchar4> pose_estimation_vis_;

  // Raycasted world-space points and normals.
  DeviceArray2D<float4> world_points_;
  DeviceArray2D<float4> world_normals_;

  DepthProcessor depth_processor_;

  RegularGridTSDF regular_grid_;

  // TODO: PoseEstimator class
  Array2D<uint8_t> board_image_;
  const int kMaxSuccessiveFailuresBeforeReset = 10;
  int num_successive_failures_ = 0;
  ProjectivePointPlaneICP icp_;

  const EuclideanTransform initial_depth_camera_from_world_;
  // Stores camera_from_world.
  std::vector<PoseFrame> depth_pose_history_;
  std::vector<PoseFrame> color_pose_history_estimated_from_depth_;

  const RGBDCameraParameters camera_params_;
  // camera_params_.depth_intrinsics_, stored as a Vector4f.
  const Vector4f depth_intrinsics_flpp_;
  const Range1f depth_range_;

  // Using the known extrinsic calibration between the color and depth
  // cameras, convert a depth camera_from_world pose to a color
  // camera_from_world pose.
  EuclideanTransform GetColorCameraFromWorld(
    const EuclideanTransform& depth_camera_from_world) const;
};

#endif  // REGULAR_GRID_FUSION_PIPELINE_H
