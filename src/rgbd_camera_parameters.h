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
#ifndef RGBD_CAMERA_PARAMETERS_H
#define RGBD_CAMERA_PARAMETERS_H

#include <core/cameras/Intrinsics.h>
#include <core/vecmath/EuclideanTransform.h>
#include <core/vecmath/Range1f.h>
#include <core/vecmath/Vector2i.h>

struct RGBDCameraParameters {

  using Intrinsics = libcgt::core::cameras::Intrinsics;
  using EuclideanTransform = libcgt::core::vecmath::EuclideanTransform;

  // Color camera.
  Vector2i color_resolution;
  Intrinsics color_intrinsics; // y axis points up.
  Range1f color_range; // For visualization only, in meters.

  // Depth camera.
  Vector2i depth_resolution;
  Intrinsics depth_intrinsics; // y axis points up.
  Range1f depth_range; // In meters.

  // Extrinsic calibration between the two cameras.
  EuclideanTransform depth_from_color;  // In meters.
  EuclideanTransform color_from_depth;  // In meters.

  // TODO(jiawen): Add a noise model.
};

#endif  // RGBD_CAMERA_PARAMETERS_H
