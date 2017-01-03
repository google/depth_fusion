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
#ifndef MULTI_STATIC_CAMERA_PIPELINE_H
#define MULTI_STATIC_CAMERA_PIPELINE_H

#include "libcgt/core/cameras/PerspectiveCamera.h"
#include "libcgt/core/geometry/TriangleMesh.h"
#include "libcgt/core/vecmath/Vector2i.h"
#include "libcgt/core/vecmath/Vector3i.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"
#include "libcgt/cuda/DeviceArray2D.h"

#include "regular_grid_tsdf.h"
#include "rgbd_camera_parameters.h"
#include "depth_processor.h"
#include "projective_point_plane_icp.h"
#include "input_buffer.h"
#include "pose_frame.h"

class MultiStaticCameraPipeline {

  using EuclideanTransform = libcgt::core::vecmath::EuclideanTransform;
  using SimilarityTransform = libcgt::core::vecmath::SimilarityTransform;

 public:

  MultiStaticCameraPipeline(
    const std::vector<RGBDCameraParameters>& camera_params,
    const std::vector<EuclideanTransform>& depth_camera_poses_cfw,
    const Vector3i& grid_resolution,
    const SimilarityTransform& world_from_grid,
    float max_tsdf_value);

  int NumCameras() const;

  const RGBDCameraParameters& GetCameraParameters(int camera_index) const;

  // TODO: oriented box class
  Box3f TSDFGridBoundingBox() const;
  const SimilarityTransform& TSDFWorldFromGridTransform() const;

  void Reset();

  void NotifyInputUpdated(int camera_index,
    bool color_updated, bool depth_updated);

  InputBuffer& GetInputBuffer(int camera_index);

  // Modified from the input...
  const DeviceArray2D<float>& GetUndistortedDepthMap(int camera_index);

  PerspectiveCamera GetDepthCamera(int camera_index) const;

  // Update the regular grid with the latest image.
  void Fuse();

  void Raycast(const PerspectiveCamera& camera,
               DeviceArray2D<float4>& world_points,
               DeviceArray2D<float4>& world_normals);

  TriangleMesh Triangulate(
    const Matrix4f& output_from_world = Matrix4f::identity()) const;

 private:
  // ----- Inputs -----
  std::vector<InputBuffer> input_buffers_;

  // ----- Intermediate buffers -----
  // Incoming raw depth frame in meters.
  std::vector<DeviceArray2D<float>> depth_meters_;
  // Incoming raw depth, undistorted.
  std::vector<DeviceArray2D<float>> undistorted_depth_meters_;

  // ----- Data structure to store the TSDF -----
  RegularGridTSDF regular_grid_;

  // ----- Processors -----
  DepthProcessor depth_processor_;

  // ----- Constants -----
  const std::vector<EuclideanTransform> depth_camera_poses_cfw_;
  const std::vector<RGBDCameraParameters> camera_params_;
  std::vector<DeviceArray2D<float2>> depth_camera_undistort_maps_;
};

#endif  // MULTI_STATIC_CAMERA_PIPELINE_H
