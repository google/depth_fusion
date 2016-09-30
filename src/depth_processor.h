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
#ifndef DEPTH_PROCESSOR_H
#define DEPTH_PROCESSOR_H

#include <core/cameras/Intrinsics.h>
#include <core/vecmath/Range1f.h>
#include <core/vecmath/Vector4f.h>
#include <CUDA/DeviceArray2D.h>

class DepthProcessor {
 public:

  using Intrinsics = libcgt::core::cameras::Intrinsics;

  DepthProcessor(const Intrinsics& depth_intrinsics,
    const Range1f& depth_range);

  void SmoothDepth(DeviceArray2D<float>& raw_depth,
    DeviceArray2D<float>& smoothed_depth);

  void EstimateNormals(DeviceArray2D<float>& smoothed_depth,
    DeviceArray2D<float4>& normals);

  const Vector4f depth_intrinsics_flpp_;
  const Range1f depth_range_;
  const int kernel_radius_ = 2;
  const float delta_z_squared_threshold_ = 0.04f;  // 40 mm for Kinect.

};

#endif  // DEPTH_PROCESSOR_H