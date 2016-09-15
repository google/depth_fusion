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
#include <CUDA/DeviceArray2D.h>

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
  RegularGridFusionPipeline(
    const RGBDCameraParameters& camera_params,
    const Vector3i& grid_resolution, float voxel_size,
    const SimilarityTransform& world_from_grid,
    // true if it's the depth camera, false if it's the color camera
    bool initial_pose_depth,
    const EuclideanTransform& initial_camera_from_world);

  void Reset();

  void NotifyInputUpdated(bool color_updated, bool depth_updated);

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

  // TODO(jiawen):
  //void Raycast(camera c)

  TriangleMesh Triangulate() const;

  // Returns CameraFromworld.
  // Each frame is directly transformed from those in DepthPoseHistory.
  // I.e., each frame has the same frame_index and timestamp, and the transform
  // is simply color_from_depth * DepthPoseHistory[i].camera_from_world.
  const std::vector<PoseFrame>& ColorPoseHistoryEstimatedDepth() const;

  // Return CameraFromWorld.
  const std::vector<PoseFrame>& DepthPoseHistory() const;

  InputBuffer input_buffer_;

  RegularGridTSDF regular_grid_;

  // Incoming depth frame in meters.
  DeviceArray2D<float> depth_meters_;
  // Incoming depth smoothed using a bilateral filter.
  DeviceArray2D<float> smoothed_depth_meters_;
  // Incoming camera-space normals, estimated from smoothed depth.
  DeviceArray2D<float4> incoming_camera_normals_;

  // Raycasted world-space points and normals.
  DeviceArray2D<float4> world_points_;
  DeviceArray2D<float4> world_normals_;
  DeviceArray2D<uchar4> icp_debug_vis_;

  // TODO(jiawen): cuda interop / store elsewhere
  Array2D<float> debug_smoothed_depth_meters_;
  Array2D<uint8_t> debug_smoothed_depth_vis_;
  Array2D<Vector4f> debug_incoming_camera_normals_;

  Array2D<float4> debug_points_world_;
  Array2D<float4> debug_normals_world_;
  Array2D<uint8x4> debug_icp_debug_vis_;

 private:

   DepthProcessor depth_processor_;

   // TODO: PoseEstimator class
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
