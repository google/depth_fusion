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
#include "depth_processor.h"

#include <cuda/Event.h>
#include <CUDA/MathUtils.h>
#include <CUDA/Rect2i.h>
#include <CUDA/ThreadMath.cuh>
#include <cuda/VecmathConversions.h>

#include "camera_math.cuh"

using libcgt::cuda::Event;
using libcgt::cuda::threadmath::threadSubscript2DGlobal;
using libcgt::cuda::contains;
using libcgt::cuda::inset;
using libcgt::cuda::Rect2i;
using libcgt::cuda::math::numBins2D;

__global__
void SmoothDepthMapKernel(KernelArray2D<const float> input,
  float2 depth_min_max,
  int kernel_radius,
  float delta_z_squared_threshold,
  KernelArray2D<float> smoothed) {
  int2 xy = threadSubscript2DGlobal();
  Rect2i valid_rect = inset(Rect2i(input.size()),
    { kernel_radius, kernel_radius });
  float z = input[xy];
  float smoothed_z = 0.0f;
  if (contains(valid_rect, xy) &&
    z >= depth_min_max.x && z <= depth_min_max.y) {

    float sum = 0.0f;
    float sum_weights = 0.0f;

    for (int dy = -kernel_radius; dy <= kernel_radius; ++dy) {
      for (int dx = -kernel_radius; dx <= kernel_radius; ++dx) {
        float z2 = input[xy + int2{dx, dy}];
        float delta_z = z2 - z;
        float delta_z_squared = delta_z * delta_z;
        if (z2 != 0 && delta_z_squared < delta_z_squared_threshold) {
          float dr2 = dx * dx + dy * dy;
          float dr = sqrt(dr2);
          // TODO(jiawen): Hacky bilateral filter without exp().
          float spatial_weight = 1.0f / (1.0f + dr);
          float range_weight = delta_z_squared_threshold - delta_z_squared;
          float weight = spatial_weight * range_weight;
          sum += weight * z2;
          sum_weights += weight;
        }
      }
    }

    if (sum_weights > 0.0f) {
      smoothed_z = sum / sum_weights;
    }
  }
  smoothed[xy] = smoothed_z;
}

__global__
void EstimateNormalsKernel(KernelArray2D<const float> depth_map,
  float4 flpp, float2 depth_min_max,
  KernelArray2D<float4> normals) {
  int2 xy = threadSubscript2DGlobal();
  float4 normal = {};

  if (xy.x < depth_map.width() - 1 && xy.y < depth_map.height() - 1) {
    float depth0 = depth_map[xy];
    int2 xy1{ xy.x + 1, xy.y };
    int2 xy2{ xy.x, xy.y + 1 };
    float depth1 = depth_map[xy1];
    float depth2 = depth_map[xy2];

    if (depth0 >= depth_min_max.x && depth0 <= depth_min_max.y &&
      depth1 >= depth_min_max.x && depth1 <= depth_min_max.y &&
      depth2 >= depth_min_max.x && depth2 <= depth_min_max.y) {

      // TODO(jiawen): can optimize this by not using CameraFromPixel and just
      // scaling x and y by z.
      float3 p0 = CameraFromPixel(xy, depth0, flpp);
      float3 p1 = CameraFromPixel(xy1, depth1, flpp);
      float3 p2 = CameraFromPixel(xy2, depth2, flpp);

      float3 dx = p1 - p0;
      float3 dy = p2 - p0;
      float3 n = cross(dx, dy);
      float lenSquared = lengthSquared(n);
      if (lenSquared > 0.0f) {
        normal = make_float4(n / sqrt(lenSquared), 1.0f);
      }
    }
  }

  normals[xy] = normal;
}

DepthProcessor::DepthProcessor(const Intrinsics& depth_intrinsics,
  const Range1f& depth_range) :
  depth_intrinsics_flpp_{ depth_intrinsics.focalLength,
    depth_intrinsics.principalPoint },
  depth_range_(depth_range) {

}

void DepthProcessor::SmoothDepth(DeviceArray2D<float>& raw_depth,
  DeviceArray2D<float>& smoothed_depth) {

  dim3 block(16, 16);
  dim3 grid = numBins2D(make_int2(raw_depth.size()), block);

  Event e;
  SmoothDepthMapKernel<<<grid, block>>>(
    raw_depth.readView(),
    make_float2(depth_range_.leftRight()),
    kernel_radius_,
    delta_z_squared_threshold_,
    smoothed_depth.writeView());
  float dtMS = e.recordStopSyncAndGetMillisecondsElapsed();
  printf("SmoothDepth took %f ms\n", dtMS);
}

void DepthProcessor::EstimateNormals(DeviceArray2D<float>& smoothed_depth,
  DeviceArray2D<float4>& normals) {
  dim3 block(16, 16);
  dim3 grid = numBins2D(make_int2(smoothed_depth.size()), block);

  Event e;
  EstimateNormalsKernel<<<grid, block>>>(
    smoothed_depth.readView(),
    make_float4(depth_intrinsics_flpp_),
    make_float2(depth_range_.leftRight()),
    normals.writeView());
  float dtMS = e.recordStopSyncAndGetMillisecondsElapsed();
  printf("EstimateNormals took %f ms\n", dtMS);
}
