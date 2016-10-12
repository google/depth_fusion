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
#ifndef RAYCAST_H
#define RAYCAST_H

#include <vector_types.h>

#include <cuda/KernelArray2D.h>
#include <cuda/KernelArray3D.h>
#include <cuda/float3x3.h>
#include <cuda/float4x4.h>

#include "regular_grid_tsdf.h"

__global__
void RaycastKernel(KernelArray3D<const TSDF> regular_grid,
  float4x4 grid_from_world, // in meters
  float4x4 world_from_grid, // in meters
  float max_tsdf_value,
  float4 flpp, // depth camera intrinsics
  float4x4 world_from_camera, // depth camera pose
  float3 eye_world, // depth camera eye in world coords
  KernelArray2D<float4> world_depth_out,
  KernelArray2D<float4> world_normal_out
);

#endif // RAYCAST_H
