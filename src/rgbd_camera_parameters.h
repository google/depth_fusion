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

#include "libcgt/core/cameras/Intrinsics.h"
#include "libcgt/core/common/Array2D.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"
#include "libcgt/core/vecmath/Range1f.h"
#include "libcgt/core/vecmath/Vector2i.h"

#include <opencv2/core/persistence.hpp>

struct CameraParameters {
  using Intrinsics = libcgt::core::cameras::Intrinsics;

  Vector2i resolution;
  // GL-style intrinsics. y-up, half-integer pixel centers.
  Intrinsics intrinsics;
  std::vector<float> dist_coeffs;
  Array2D<Vector2f> undistortion_map; // y axis points up.

  // TODO: move this into depth parameters only
  Range1f depth_range; // In meters.
};

struct RGBDCameraParameters {
  using EuclideanTransform = libcgt::core::vecmath::EuclideanTransform;

  CameraParameters color;
  CameraParameters depth;

  // Extrinsic calibration between the two cameras.
  EuclideanTransform depth_from_color;  // In meters.
  EuclideanTransform color_from_depth;  // In meters.

  // TODO: Add a noise model.

  // Using the known extrinsic calibration between the color and depth
  // cameras, convert a depth camera_from_world pose to a color
  // camera_from_world pose.
  EuclideanTransform ConvertToColorCameraFromWorld(
    const EuclideanTransform& depth_camera_from_world) const;

  // Using the known extrinsic calibration between the color and depth
  // cameras, convert a color camera_from_world pose to a depth
  // camera_from_world pose.
  EuclideanTransform ConvertToDepthCameraFromWorld(
    const EuclideanTransform& color_camera_from_world) const;
};

// Load a CameraParameters object from an already-opened OpenCV FileStorage
// object. Reads the keys:
// <namePrefix>{ImageSize, CameraMatrix_gl, DistCoeffs}.
CameraParameters LoadCameraIntrinsics(const cv::FileStorage& fs,
	const std::string& namePrefix);

// Load a RGBDCameraParameters from a directory containing:
// <dir>/stereo_calibration.yaml,
// <dir>/color_undistort_map_gl.pfm2
// <dir>/depth_undistort_map_gl.pfm2
// Calls LoadCameraIntrinsics() on the YAML file with "color" and 'depth" as
// prefixes for the intrinsics. Also reads "colorFromDepth_gl" and
// "depthFromColor_gl" for extrinsics.
RGBDCameraParameters LoadRGBDCameraParameters(const std::string& dir);

#endif  // RGBD_CAMERA_PARAMETERS_H
