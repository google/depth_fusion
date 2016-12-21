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

#include <cuda/float4x4.h>
#include <cuda/MathUtils.h>
#include <cuda/ThreadMath.cuh>

#include "camera_math.cuh"

using libcgt::cuda::threadmath::threadSubscript2DGlobal;
using libcgt::cuda::contains;
using libcgt::cuda::math::roundToInt;

#define USE_BILINEAR_DEPTH_SAMPLING 1

// TODO(jiawen): move this into class regular_grid_tsdf.
__global__
void FuseKernel(
  float4x4 world_from_grid,
  float max_tsdf_value,
  float4 flpp,
  float2 zMinMax,
  float4x4 camera_from_world,
  KernelArray2D<const float> depth_data,
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

#if USE_BILINEAR_DEPTH_SAMPLING
    // Use bilinear interpolation.
    int2 uv_int = roundToInt(uv);

    if (voxel_center_camera.z > 0 ||
        uv_int.x < 0 || uv_int.y < 0 ||
	uv_int.x + 1 >= depth_data.size().x || uv_int.y + 1 >= depth_data.size().y) {
      continue;
    }

    float2 fuv = uv - float2{uv_int.x, uv_int.y};
    float d00 = depth_data[uv_int + int2{0, 0}];
    float d10 = depth_data[uv_int + int2{1, 0}];
    float d01 = depth_data[uv_int + int2{0, 1}];
    float d11 = depth_data[uv_int + int2{1, 1}];

    // Check if all depth measurements are within the range.
    if (min(min(d00,d01),min(d10,d11)) < zMinMax.x ||
	max(max(d00,d01),max(d10,d11)) > zMinMax.y) {
      continue;
    }

    float image_depth = (1 - fuv.y) * (d00 * (1 - fuv.x) + fuv.x * d10) +
	                fuv.y * (d01 * (1 - fuv.x) + fuv.x * d11);
#else
    // Use nearest sampling on the depth texture.
    int2 uv_int = roundToInt(uv - float2{ 0.5f, 0.5f });

    if (voxel_center_camera.z > 0 ||
      !contains(depth_data.size(), uv_int)) {
      continue;
    }

    float image_depth = depth_data[uv_int];
#endif

    if (image_depth < zMinMax.x || image_depth > zMinMax.y) {
      continue;
    }

    // Compute standard deviation of depth in a squared patch around the sample.
    // This value is can be used a proxy for the weight of each sample,
    // so the samples around depth discontinuties or grazing angles can be
    // downsampled to avoid exaggerated space carving artifacts.
    const int kDepthVarRadius = 3;

    if (uv_int.x - kDepthVarRadius < 0 || uv_int.y - kDepthVarRadius < 0 ||
	uv_int.x + kDepthVarRadius >= depth_data.size().x ||
	uv_int.y + kDepthVarRadius  >=depth_data.size().y) {
      continue;
    }

    float sum_d = 0.0f, sum_d2 = 0.0f;
    for (int dx = -kDepthVarRadius; dx <= kDepthVarRadius; dx++) {
      for (int dy = -kDepthVarRadius; dy <= kDepthVarRadius; dy++) {
        float d = depth_data[uv_int + int2{dx, dy}];
        sum_d += d;
        sum_d2 += d * d;
      }
    }
    const int var_n = (2 * kDepthVarRadius + 1) * (2 * kDepthVarRadius + 1);
    float var_d = sum_d2 / var_n - sum_d * sum_d / var_n / var_n;
    float std_d = sqrt(var_d);

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

      // Weights can be larger than 1.0, the absolute value only matters in
      // the marching cubes algorithm, where a threshold on the weight decides
      // where to create a triangle.
      // The weight is higher the less standard deviation in a square patch
      // around surface.
      float weight = 10.0f * min(0.1f / std_d, 1.0f);

      regular_grid[{ij.x, ij.y, k}].Update(dz, weight, max_tsdf_value);
    }
  }
}
