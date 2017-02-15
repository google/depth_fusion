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

#include "libcgt/core/cameras/PerspectiveCamera.h"
#include "libcgt/core/geometry/TriangleMesh.h"
#include "libcgt/core/vecmath/Vector2i.h"
#include "libcgt/core/vecmath/Vector3i.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"
#include "libcgt/cuda/DeviceArray2D.h"

#include "aruco/aruco_pose_estimator.h"
#include "aruco/cube_fiducial.h"
#include "aruco/single_marker_fiducial.h"
#include "regular_grid_tsdf.h"
#include "rgbd_camera_parameters.h"
#include "depth_processor.h"
#include "input_buffer.h"
#include "pipeline_data_type.h"
#include "pose_estimation_method.h"
#include "pose_frame.h"
#include "projective_point_plane_icp.h"

struct PoseEstimatorOptions {
  PoseEstimationMethod method =
    PoseEstimationMethod::COLOR_ARUCO_AND_DEPTH_ICP;

  // Required if method is DEPTH_ICP.
  PoseFrame initial_pose = PoseFrame{};

  // Required if method is PRECOMPUTED or PRECOMPUTED_REFINE_WITH_DEPTH_ICP.
  std::vector<PoseFrame> precomputed_path;
};

class RegularGridFusionPipeline : public QObject {
 Q_OBJECT

  using EuclideanTransform = libcgt::core::vecmath::EuclideanTransform;
  using SimilarityTransform = libcgt::core::vecmath::SimilarityTransform;

 public:

  RegularGridFusionPipeline(
    const RGBDCameraParameters& camera_params,
    const Vector3i& grid_resolution,
    const SimilarityTransform& world_from_grid,
    const PoseEstimatorOptions& pose_estimator_options);

  void Reset();

  const RGBDCameraParameters& GetCameraParameters() const;
  const CubeFiducial& GetArucoCubeFiducial() const;
  const SingleMarkerFiducial& GetArucoSingleMarkerFiducial() const;

  // Get a mutable reference to the input buffer. Clients should update the
  // values in the input buffer then call NotifyInputUpdated();
  InputBuffer& GetInputBuffer();

  // Get a read-only view of the latest color pose estimator's visualization.
  // TODO: this buffer is y-up but BGR format.
  Array2DReadView<uint8x3> GetColorPoseEstimatorVisualization() const;

  void NotifyColorUpdated();

  void NotifyDepthUpdated();

  // Returns the TSDF grid's axis aligned bounding box.
  // (0, 0, 0) --> Resolution().
  // TODO: implement a simple oriented box class.
  Box3f TSDFGridBoundingBox() const;

  // The transformation that yields world coordinates (in meters) from
  // grid coordinates [0, resolution]^3 (in samples).
  const SimilarityTransform& TSDFWorldFromGridTransform() const;

  PerspectiveCamera ColorCamera() const;
  PerspectiveCamera DepthCamera() const;

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

   // Try to estimate the rgbd camera pose using the latest color frame of the
   // pipeline's input buffer. If it succeeded, it will be appended to the
   // pose history and returns true. Otherwise, returns false.
   bool UpdatePoseWithColorCamera();

   // Try to estimate the rgbd camera pose using the latest depth frame of the
   // pipeline's input buffer. If it succeeded, it will be appended to the
   // pose history and returns true. Otherwise, returns false.
   bool UpdatePoseWithDepthCamera();

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

  // TODO: move this into PoseEstimator?
  const int kMaxSuccessiveFailuresBeforeReset = 1000;
  int num_successive_failures_ = 0;
  ProjectivePointPlaneICP icp_;

  CubeFiducial aruco_cube_fiducial_;
  SingleMarkerFiducial aruco_single_marker_fiducial_;
  ArucoPoseEstimator aruco_pose_estimator_;
  // Visualization of the last pose estimate, y up.
  Array2D<uint8x3> aruco_vis_;

  PoseEstimatorOptions pose_estimator_options_;
  bool is_first_depth_frame_ = true;
  std::vector<PoseFrame> pose_history_;

  const RGBDCameraParameters camera_params_;
  // camera_params_.depth_intrinsics_, stored as a Vector4f.
  const Vector4f depth_intrinsics_flpp_;
  const Range1f depth_range_;
};

#endif  // REGULAR_GRID_FUSION_PIPELINE_H
