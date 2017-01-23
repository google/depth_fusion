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
#include "artoolkitplus_pose_estimator.h"

#include <cassert>

#include "libcgt/core/common/ArrayUtils.h"
#include "libcgt/core/time/TimeUtils.h"
#include "libcgt/core/vecmath/Matrix4f.h"

#include "rgbd_camera_parameters.h"

namespace {

// Convert the result of a pose estimate (from, e.g.,
// arGetTransMatMultiSquare(), into a Matrix4f representing a camera from world
// transformation with y-up.
Matrix4f ConvertToCameraFromWorldMeters(const ARFloat matrix[3][4]) {
  ARFloat view_matrix[16];
  view_matrix[0 + 0*4] = matrix[0][0]; // R1C1
	view_matrix[0 + 1*4] = matrix[0][1]; // R1C2
	view_matrix[0 + 2*4] = matrix[0][2];
	view_matrix[0 + 3*4] = matrix[0][3];
  view_matrix[1 + 0*4] = matrix[1][0]; // R2
	view_matrix[1 + 1*4] = matrix[1][1];
	view_matrix[1 + 2*4] = matrix[1][2];
	view_matrix[1 + 3*4] = matrix[1][3];
	view_matrix[2 + 0*4] = matrix[2][0]; // R3
	view_matrix[2 + 1*4] = matrix[2][1];
	view_matrix[2 + 2*4] = matrix[2][2];
	view_matrix[2 + 3*4] = matrix[2][3];
	view_matrix[3 + 0*4] = 0.0;
	view_matrix[3 + 1*4] = 0.0;
	view_matrix[3 + 2*4] = 0.0;
	view_matrix[3 + 3*4] = 1.0;

  Matrix4f output;
  for (int k = 0; k < 16; ++k) {
    output[k] = static_cast<float>(view_matrix[k]);
  }
  // Convert from millimeters to meters.
  output.column3.xyz *= 0.001f;

  const Matrix4f rot90 = Matrix4f::rotateX(static_cast<float>(M_PI_2));
  return output * rot90;
}

}

ARToolkitPlusPoseEstimator::ARToolkitPlusPoseEstimator(
  const CameraParameters& params, const std::string& marker_cube_filename) :
  rgbx_(params.resolution),
  tracker_(
    params.resolution.x, params.resolution.y,
    6,     // maxImagePatterns
    6, 6   // pattern width and height
  ) {
  tracker_.initMarkerConfig(marker_cube_filename.c_str());
  tracker_.initCamera(params.resolution.x, params.resolution.y,
    static_cast<ARFloat>(params.intrinsics.focalLength.x),
    static_cast<ARFloat>(params.intrinsics.focalLength.y),
    static_cast<ARFloat>(params.intrinsics.principalPoint.x),
    // Flip y since ARToolkit uses y-down.
    static_cast<ARFloat>(
      params.resolution.y - params.intrinsics.principalPoint.y),
    static_cast<ARFloat>(params.dist_coeffs[0]),
    static_cast<ARFloat>(params.dist_coeffs[1]),
    static_cast<ARFloat>(params.dist_coeffs[2]),
    static_cast<ARFloat>(params.dist_coeffs[3]),
    static_cast<ARFloat>(params.dist_coeffs[4]),
    static_cast<ARFloat>(params.depth_range.left()),
    static_cast<ARFloat>(params.depth_range.right()));
  tracker_.setUseDetectLite(false);

  // TODO: flag
  tracker_.activateAutoThreshold(true);

  // TODO: necessary?
  tracker_.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_RGBA);
}

ARToolkitPlusPoseEstimator::Result ARToolkitPlusPoseEstimator::EstimatePose(
  Array2DReadView<uint8x3> bgr) {
  Result result;
  if (!bgr.packed()) {
    return result;
  }

  // Ugh need to convert anyway!.
  auto t0 = std::chrono::high_resolution_clock::now();

  libcgt::core::arrayutils::map(bgr, rgbx_.writeView(),
    [&] (uint8x3 bgr)
    {
      return uint8x4{bgr.z, bgr.y, bgr.x, 255};
    });

  tracker_.calc(reinterpret_cast<const uint8_t*>(rgbx_.pointer()));

  result.nMarkers = tracker_.getNumDetectedMarkers();
  if (result.nMarkers > 0) {
    Matrix4f camera_from_world = ConvertToCameraFromWorldMeters(
      tracker_.getMultiMarkerConfig()->trans);

    result.world_from_camera = inverse(
      EuclideanTransform::fromMatrix(camera_from_world));
    result.valid = true;
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  if(result.valid) {
    printf("ARToolkitPlus: succeeded, nMarkers = %d, error = %lf.\n",
      result.nMarkers, result.error);
  } else {
    printf("ARToolkitPlus: failed.\n");
  }
  printf( "ARToolkitPlus: pose estimation took %lld ms\n",
    libcgt::core::time::dtMS(t0, t1));
  return result;
}
