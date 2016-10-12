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
#include "raycast.h"

#include <cuda/Box3f.h>
#include <cuda/Event.h>
#include <cuda/MathUtils.h>
#include <cuda/ThreadMath.cuh>

#include "camera_math.cuh"
#include "tsdf.h"

using libcgt::cuda::contains;
using libcgt::cuda::Event;
using libcgt::cuda::math::floorToInt;
using libcgt::cuda::threadmath::threadSubscript2DGlobal;

__inline__ __device__ __host__
float2 half2()
{
    return make_float2(0.5f);
}

__inline__ __device__ __host__
float3 half3()
{
    return make_float3(0.5f);
}

__inline__ __device__ __host__
float2 one2()
{
    return make_float2(1.0f);
}

__inline__ __device__ __host__
float3 one3()
{
    return make_float3(1.0f);
}

// TODO: consider optimizing this by removing all boundary checks and
// adjusting the kernel.

// TODO: make this a method.
// TODO: easy way to enforce boundary conditions:
// clamp x to 0.5, width - 0.5,, etc.
// But that's not useful for SDFs! When you want to know when you're invalid.
// Assumes the grid_point is valid (coords are within the extents).
__inline__ __device__
float2 TrilinearSample(KernelArray3D<const TSDF> regular_grid,
  float3 grid_coords, float max_tsdf_value) {
  // For trilinear interpolation, the valid range is between [0.5, size - 0.5].
  libcgt::cuda::Box3f valid_box(half3(),
    make_float3(regular_grid.size()) - one3());
  if (!valid_box.contains(grid_coords)) {
    return{ 0.0f, 0.0f };
  }

  //
  float3 integer_grid_coords = grid_coords - half3();
  int3 p_000 = floorToInt(integer_grid_coords);
  float3 t = fracf(integer_grid_coords);
  int3 p_100 = { p_000.x + 1, p_000.y,     p_000.z     };
  int3 p_010 = { p_000.x    , p_000.y + 1, p_000.z     };
  int3 p_110 = { p_000.x + 1, p_000.y + 1, p_000.z     };
  int3 p_001 = { p_000.x    , p_000.y    , p_000.z + 1 };
  int3 p_101 = { p_000.x + 1, p_000.y    , p_000.z + 1 };
  int3 p_011 = { p_000.x    , p_000.y + 1, p_000.z + 1 };
  int3 p_111 = { p_000.x + 1, p_000.y + 1, p_000.z + 1 };

  TSDF v_000 = regular_grid[p_000];
  TSDF v_100 = regular_grid[p_100];
  TSDF v_010 = regular_grid[p_010];
  TSDF v_110 = regular_grid[p_110];
  TSDF v_001 = regular_grid[p_001];
  TSDF v_101 = regular_grid[p_101];
  TSDF v_011 = regular_grid[p_011];
  TSDF v_111 = regular_grid[p_111];

  // TODO(jiawen): can save a branch by multiplying by weight, or maybe storing
  // pre-multiplied.
  if (v_000.Weight() == 0 || v_100.Weight() == 0 ||
    v_010.Weight() == 0 || v_110.Weight() == 0 ||
    v_001.Weight() == 0 || v_101.Weight() == 0 ||
    v_011.Weight() == 0 || v_111.Weight() == 0) {
    return{ 0.0f, 0.0f };
  }

  // Trilerp, ignoring weights.
  // TODO(jiawen): weighted sdf?
  float d_000 = v_000.Distance(max_tsdf_value);
  float d_100 = v_100.Distance(max_tsdf_value);
  float d_010 = v_010.Distance(max_tsdf_value);
  float d_110 = v_110.Distance(max_tsdf_value);
  float d_001 = v_001.Distance(max_tsdf_value);
  float d_101 = v_101.Distance(max_tsdf_value);
  float d_011 = v_011.Distance(max_tsdf_value);
  float d_111 = v_111.Distance(max_tsdf_value);

  // Lerp in x.
  float d_l00 = lerp(d_000, d_100, t.x);
  float d_l10 = lerp(d_010, d_110, t.x);
  float d_l01 = lerp(d_000, d_101, t.x);
  float d_l11 = lerp(d_010, d_111, t.x);

  // Lerp in y.
  float d_ll0 = lerp(d_l00, d_l10, t.y);
  float d_ll1 = lerp(d_l01, d_l11, t.y);

  // Lerp in z.
  return { lerp(d_ll0, d_ll1, t.z), 1.0f };
}

// TODO(jiawen): optimized version without checks?
__inline__ __device__
float4 TrilinearSampleNormal(KernelArray3D<const TSDF> regular_grid,
  float3 grid_coords, float max_tsdf_value) {
  float3 dx3 = { 1, 0, 0 };
  float3 dy3 = { 0, 1, 0 };
  float3 dz3 = { 0, 0, 1 };

  float2 d_000 = TrilinearSample(regular_grid, grid_coords, max_tsdf_value);
  float2 d_100 = TrilinearSample(regular_grid, grid_coords + dx3, max_tsdf_value);
  float2 d_010 = TrilinearSample(regular_grid, grid_coords + dy3, max_tsdf_value);
  float2 d_001 = TrilinearSample(regular_grid, grid_coords + dz3, max_tsdf_value);
  bool valid = d_000.y != 0 && d_100.y != 0 && d_010.y != 0 && d_001.y != 0;

  float4 normal_out = {};

  float3 normal = {
    d_100.x - d_000.x,
    d_010.x - d_000.x,
    d_001.x - d_000.x,
  };
  float len = length(normal);
  if (valid && len > 0) {
    normal_out = make_float4(normal / len, 1.0f);
  }

  return normal_out;
}

#define kTEpsilon 2.0f
#define kTStepSize 1.0f

__global__
void RaycastKernel(KernelArray3D<const TSDF> regular_grid,
  float4x4 grid_from_world,
  float4x4 world_from_grid,
  float max_tsdf_value,
  float4 flpp,
  float4x4 world_from_camera,
  float3 eye_world,
  KernelArray2D<float4> world_points_out,
  KernelArray2D<float4> world_normals_out) {
  // TODO(jiawen): simplify this logic with a "bool valid" flag.
  float4 world_point = {};
  float4 world_normal = {};

  // Cast a ray for each pixel.
  int2 xy = threadSubscript2DGlobal();
  if (!contains(world_points_out.size(), xy)) {
    world_points_out[xy] = world_point;
    world_normals_out[xy] = world_normal;
    return;
  }

  float3 dir_grid = normalize(transformVector(grid_from_world,
    transformVector(world_from_camera, CameraDirectionFromPixel(xy, flpp))));

  // TODO(jiawen): make this a method, or pass it in directly
  float3 eye_grid = transformPoint(grid_from_world, eye_world);

  // Pick a starting point: intersect the ray with the grid bounding box.
  float t_near;
  float t_far;
  // TODO(jiawen): intersect with a grid that's 1 voxel smaller.
  libcgt::cuda::Box3f bbox_grid(regular_grid.size());
  bool intersected = libcgt::cuda::intersectLine(eye_grid, dir_grid,
    bbox_grid, t_near, t_far);

  if (!intersected) {
    world_points_out[xy] = world_point;
    world_normals_out[xy] = world_normal;
    return;
  }

  // If the near starting point is behind the eye, clamp it to the eye.
  // If it's in front of the eye, then start there.
  // But we don't want to start directly on a face, so add epsilon to it.
  float t_start = fmaxf(0, t_near) + kTEpsilon;

  // Likewise, the end point should not be on a face.
  float t_end = fmaxf(0, t_far) - kTEpsilon;

  int num_iterations = floorToInt((t_end - t_start) / kTStepSize);

  // Iterate until we exit, or found a surface.
  bool found_surface = false;

  float prev_t;
  float3 prev_coords_grid = {};
  float2 prev_sdf = {};

  float curr_t = t_near;
  float3 curr_coords_grid = eye_grid + curr_t * dir_grid;
  float2 curr_sdf =
    TrilinearSample(regular_grid, curr_coords_grid, max_tsdf_value);

  for (int i = 1; i < num_iterations; ++i) {
    prev_t = curr_t;
    prev_coords_grid = curr_coords_grid;
    prev_sdf = curr_sdf;

    curr_t = prev_t + kTStepSize;
    curr_coords_grid = eye_grid + curr_t * dir_grid;
    curr_sdf = TrilinearSample(regular_grid, curr_coords_grid, max_tsdf_value);

    // Both samples are valid, and it's a positive to negative zero crossing.
    if (prev_sdf.y > 0 && curr_sdf.y > 0 &&
      prev_sdf.x > 0 && curr_sdf.x < 0) {
      found_surface = true;
      break;
    }
  }

  if (found_surface) {
    // How far should I interpolate between the SDF values?
    float alpha = prev_sdf.x / (prev_sdf.x - curr_sdf.x);

    // Use it to lerp t itself to get a better estimate of the zero crossing.
    float t_at_surface = lerp(prev_t, curr_t, alpha);

    float3 surface_point_grid = eye_grid + t_at_surface * dir_grid;

    // Convert to world space.
    // TODO(jiawen): make this a method
    world_point = make_float4(
      transformPoint(world_from_grid, surface_point_grid), 1.0f);
    float4 grid_normal = TrilinearSampleNormal(regular_grid,
      surface_point_grid, max_tsdf_value);
    if (grid_normal.w > 0) {
      // TODO(jiawen): the gradient is weird and only needs the rotation part
      // of the world_from_grid transformation. It's because we store world
      // distances in the grid.
      world_normal = make_float4(
        normalize(transformVector(world_from_grid, make_float3(grid_normal))),
        1.0f);
    }
  }

  world_points_out[xy] = world_point;
  world_normals_out[xy] = world_normal;
}
