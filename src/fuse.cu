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
#include "fuse.h"

#include "libcgt/cuda/float4x4.h"
#include "libcgt/cuda/MathUtils.h"
#include "libcgt/cuda/ThreadMath.cuh"

#include "camera_math.cuh"

using libcgt::cuda::threadmath::threadSubscript2DGlobal;
using libcgt::cuda::contains;
using libcgt::cuda::math::roundToInt;

__global__
void FuseKernel(
  float4x4 world_from_grid,
  float max_tsdf_value,
  float4 flpp,
  float2 depth_min_max,
  float4x4 camera_from_world,
  KernelArray2D<const float> depth_map,
  KernelArray3D<TSDF> regular_grid) {

  int2 ij = threadSubscript2DGlobal();

  // Sweep over the entire volume.
  for (int k = 0; k < regular_grid.depth(); ++k) {
    // Find the voxel center.
    // TODO(jiawen): write a helper function that takes in a subscript
    float4 voxel_center_world = make_float4(
      transformPoint(
        world_from_grid, float3{ij.x + 0.5f, ij.y + 0.5f, k + 0.5f}),
      1.0f);

    // Project it into camera coordinates.
    // camera_from_world uses OpenGL conventions,
    // so depth is a negative number if it's in front of the camera.
    float4 voxel_center_camera =
      camera_from_world * voxel_center_world;
    float2 uv = make_float2(PixelFromCamera(make_float3(voxel_center_camera),
      flpp));
    int2 uv_int = roundToInt(uv - float2{ 0.5f, 0.5f });

    if (voxel_center_camera.z > 0 ||
      !contains(depth_map.size(), uv_int)) {
      continue;
    }

    float image_depth = depth_map[uv_int];
    if (image_depth < depth_min_max.x || image_depth > depth_min_max.y) {
      continue;
    }

    // Compute dz, the signed distance between the voxel center and the
    // surface observation.
    //
    // Note that we flip the sign on z to get "depth", where positive numbers
    // are in front of the camera.
    //
    // The sign convention of the distance field is so that voxels in front of
    // the surface is positive (and voxels behind are negative).
    float voxel_center_depth = -voxel_center_camera.z;
    float dz = image_depth - voxel_center_depth;

    // Now integrate data in carefully:
    // Consider 3 cases:
    // dz < -max_tsdf_value: the voxel is behind the observation and out of the
    //   truncation region. Therefore, do nothing.
    // dz \in [-max_tsdf_value, 0]: the voxel is behind the observation and
    //   within the truncation region. Integrate.
    // dz > 0: the voxel is in front of the observation. Integrate... but if
    //   the voxel is really far in front, we don't want to put in a large
    //   value. Instead, clamp it to max_tsdf_value.
    if (dz >= -max_tsdf_value) {
      // Ignore the voxel when it is far behind.
      // Clamp to the TSDF range.
      dz = min(dz, max_tsdf_value);
      const float weight = 1.0f;

      regular_grid[{ij.x, ij.y, k}].Update(dz, weight, max_tsdf_value);
    }
  }
}

namespace {
  constexpr int kNumDepthMaps = 3;
}

__global__
void FuseMultipleKernel(
  float4x4 world_from_grid,
  float max_tsdf_value,
  CalibratedPosedDepthCamera depth_camera0,
  CalibratedPosedDepthCamera depth_camera1,
  CalibratedPosedDepthCamera depth_camera2,
  KernelArray2D<const float> depth_map0,
  KernelArray2D<const float> depth_map1,
  KernelArray2D<const float> depth_map2,
  KernelArray3D<TSDF> regular_grid) {

  CalibratedPosedDepthCamera depth_camera[] =
  {
    depth_camera0,
    depth_camera1,
    depth_camera2
  };
  KernelArray2D<const float> depth_maps[] =
  {
    depth_map0,
    depth_map1,
    depth_map2
  };

  int2 ij = threadSubscript2DGlobal();

  // Sweep over the entire volume.
  for (int k = 0; k < regular_grid.depth(); ++k) {
    // Find the voxel center.
    // TODO(jiawen): write a helper function that takes in a subscript
    float4 voxel_center_world = make_float4(
      transformPoint(
        world_from_grid, float3{ij.x + 0.5f, ij.y + 0.5f, k + 0.5f}),
      1.0f);

    for (int c = 0; c < kNumDepthMaps; ++c) {
      // Project it into camera coordinates.
      // camera_from_world uses OpenGL conventions,
      // so depth is a negative number if it's in front of the camera.
      float4 voxel_center_camera =
        depth_camera[c].camera_from_world * voxel_center_world;
      float2 uv = make_float2(
        PixelFromCamera(make_float3(voxel_center_camera), depth_camera[c].flpp));
      int2 uv_int = roundToInt(uv - float2{0.5f, 0.5f});

      if (voxel_center_camera.z > 0 ||
        !contains( depth_maps[c].size(), uv_int)) {
        continue;
      }

      float image_depth = depth_maps[c][uv_int];
      if (image_depth < depth_camera[c].depth_min_max.x ||
        image_depth > depth_camera[c].depth_min_max.y) {
        continue;
      }

      // Compute dz, the signed distance between the voxel center and the
      // surface observation.
      //
      // Note that we flip the sign on z to get "depth", where positive numbers
      // are in front of the camera.
      //
      // The sign convention of the distance field is so that voxels in front of
      // the surface is positive (and voxels behind are negative).
      float voxel_center_depth = -voxel_center_camera.z;
      float dz = image_depth - voxel_center_depth;

      // Now integrate data in carefully:
      // Consider 3 cases:
      // dz < -max_tsdf_value: the voxel is behind the observation and out of the
      //   truncation region. Therefore, do nothing.
      // dz \in [-max_tsdf_value, 0]: the voxel is behind the observation and
      //   within the truncation region. Integrate.
      // dz > 0: the voxel is in front of the observation. Integrate... but if
      //   the voxel is really far in front, we don't want to put in a large
      //   value. Instead, clamp it to max_tsdf_value.
      if (dz >= -max_tsdf_value) {
        // Ignore the voxel when it is far behind.
        // Clamp to the TSDF range.
        dz = min(dz, max_tsdf_value);
        const float weight = 1.0f;

        regular_grid[{ij.x, ij.y, k}].Update(dz, weight, max_tsdf_value);
      }
    }
  }
}
