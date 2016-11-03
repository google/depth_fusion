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

#include <QObject>

#include <core/cameras/PerspectiveCamera.h>
#include <core/geometry/TriangleMesh.h>
#include <core/vecmath/Vector2i.h>
#include <core/vecmath/Vector3i.h>
#include <core/vecmath/EuclideanTransform.h>
#include <cuda/DeviceArray2D.h>

#include "artoolkit_pose_estimator.h"
#include "regular_grid_tsdf.h"
#include "rgbd_camera_parameters.h"
#include "depth_processor.h"
#include "input_buffer.h"
#include "pipeline_data_type.h"
#include "pose_frame.h"
#include "projective_point_plane_icp.h"

class RegularGridFusionPipeline : public QObject {
 Q_OBJECT

  using EuclideanTransform = libcgt::core::vecmath::EuclideanTransform;
  using SimilarityTransform = libcgt::core::vecmath::SimilarityTransform;

 public:

  RegularGridFusionPipeline(
    const RGBDCameraParameters& camera_params,
    const Vector3i& grid_resolution,
    const SimilarityTransform& world_from_grid,
    const PoseFrame& initial_pose);

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

  bool UpdatePoseWithColorCamera();

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
  const std::vector<PoseFrame>& PoseHistory() const;

  const DeviceArray2D<float>& SmoothedDepthMeters() const;

  // In camera space.
  const DeviceArray2D<float4>& SmoothedIncomingNormals() const;

  const DeviceArray2D<uchar4>& PoseEstimationVisualization() const;

  // In world space.
  const DeviceArray2D<float4>& RaycastNormals() const;

 signals:

  // Notify subscribers that data flowing through the pipeline has changed.
  void dataChanged(PipelineDataType type);

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
  PoseFrame last_raycast_pose_ = {};
  DeviceArray2D<float4> world_points_;
  DeviceArray2D<float4> world_normals_;

  DepthProcessor depth_processor_;

  RegularGridTSDF regular_grid_;

  // TODO: PoseEstimator class
  const int kMaxSuccessiveFailuresBeforeReset = 1000;
  int num_successive_failures_ = 0;
  ProjectivePointPlaneICP icp_;
  ARToolkitPoseEstimator color_pose_estimator_;

  const PoseFrame initial_pose_;

  // Stores camera_from_world.
  std::vector<PoseFrame> pose_history_;

  const RGBDCameraParameters camera_params_;
  // camera_params_.depth_intrinsics_, stored as a Vector4f.
  const Vector4f depth_intrinsics_flpp_;
  const Range1f depth_range_;
};

#endif  // REGULAR_GRID_FUSION_PIPELINE_H
