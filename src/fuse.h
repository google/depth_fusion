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
#ifndef FUSE_H
#define FUSE_H

#include <vector_types.h>

#include "libcgt/cuda/float4x4.h"
#include "libcgt/cuda/KernelArray2D.h"
#include "libcgt/cuda/KernelArray3D.h"

#include "calibrated_posed_depth_camera.h"
#include "regular_grid_tsdf.h"

__global__
void FuseKernel(
  float4x4 world_from_grid,
  float max_tsdf_value,
  float4 flpp,
  float2 depth_min_max,
  float4x4 camera_from_world,
  KernelArray2D<const float> depth_map,
  KernelArray3D<TSDF> regular_grid);

// TODO: replace CalibratedPosedDepthCamera with something in __constant__
// memory.
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
  KernelArray3D<TSDF> regular_grid);

#endif // FUSE_H
