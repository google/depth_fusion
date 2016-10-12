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
#ifndef PROJECTIVE_POINT_PLANE_ICP_H
#define PROJECTIVE_POINT_PLANE_ICP_H

#include <core/cameras/Intrinsics.h>
#include <core/vecmath/EuclideanTransform.h>
#include <core/vecmath/Range1f.h>
#include <core/vecmath/Vector4f.h>
#include <cuda/DeviceArray2D.h>

#include "icp_least_squares_data.h"

#include <vector>

class ProjectivePointPlaneICP {
 public:

  using EuclideanTransform = libcgt::core::vecmath::EuclideanTransform;
  using Intrinsics = libcgt::core::cameras::Intrinsics;

  struct Result {
    bool valid = false;
    // The number of samples used to estimate this result.
    // If is 0, ICP failed and this result is invalid.
    int num_samples = 0;

    // TODO: reason: not enough matches, failed to translation and rotation tests, etc

    EuclideanTransform world_from_camera;
  };

  ProjectivePointPlaneICP(const Vector2i& depth_resolution,
    const Intrinsics& depth_intrinsics, const Range1f& depth_range);

  __host__
  Result EstimatePose(
    DeviceArray2D<float>& incoming_depth,
    DeviceArray2D<float4>& incoming_normals,
    const EuclideanTransform& world_from_camera,
    DeviceArray2D<float4>& world_points,
    DeviceArray2D<float4>& world_normals,
    DeviceArray2D<uchar4>& debug_vis);

 private:

   const Vector4f depth_intrinsics_flpp_;
   const Range1f depth_range_;

   //const int kMinNumSamples = 30000;
   const int kMinNumSamples = 300;
   const int kNumIterations = 15;
   const int kImageGuardBand = 16;
   const float kMaxDistanceForMatch = 0.1f;
   const float kMinDotProductForMatch = 0.7f;

   // Reject if translation > kMaxTranslation meters;
   const float kMaxTranslation = 0.15f;
   // Reject if rotation > kMaxRotationRadians;
   const float kMaxRotationRadians = 0.1745f; // 10 degrees

   DeviceArray2D<ICPLeastSquaresData> icp_data_;
};

#endif
