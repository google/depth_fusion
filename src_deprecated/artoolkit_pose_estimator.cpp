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
#include "artoolkit_pose_estimator.h"

#include <cassert>

#include <AR/gsub_lite.h>

#include "libcgt/core/vecmath/Matrix4f.h"

#include "rgbd_camera_parameters.h"

namespace {

// Convert intrinsics from our internal format to ARParam.
ARParam Convert(const CameraParameters& input) {
  ARParam output = {};
  output.dist_function_version = 4;
  output.xsize = input.resolution.x;
  output.ysize = input.resolution.y;

  output.dist_factor[0] = static_cast<ARdouble>(input.dist_coeffs[0]); // k1.
  output.dist_factor[1] = static_cast<ARdouble>(input.dist_coeffs[1]); // k2.
  output.dist_factor[2] = static_cast<ARdouble>(input.dist_coeffs[2]); // p1.
  output.dist_factor[3] = static_cast<ARdouble>(input.dist_coeffs[3]); // p2.
  output.dist_factor[4] =
    static_cast<ARdouble>(input.intrinsics.focalLength.x);
  output.dist_factor[5] =
    static_cast<ARdouble>(input.intrinsics.focalLength.y);
  output.dist_factor[6] =
    static_cast<ARdouble>(input.intrinsics.principalPoint.x);
  // y points up in CameraParameters.
  // TODO: might be able to get away with keep the image with y up.
  output.dist_factor[7] =
    static_cast<ARdouble>(input.resolution.y) -
    static_cast<ARdouble>(input.intrinsics.principalPoint.y);

  // ARToolkit has a notion of "scale factor", where the video may be captured
  // at a resolution different from that of the camera calibration.
  output.dist_factor[8] = static_cast<ARdouble>(1.0);

  // ARToolkit keeps its matrices as mat[row][col].
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      output.mat[i][j] = static_cast<ARdouble>(0.0);
    }
  }
  output.mat[2][2] = static_cast<ARdouble>(1.0);

  output.mat[0][0] = static_cast<ARdouble>(input.intrinsics.focalLength.x);
  output.mat[0][2] = static_cast<ARdouble>(input.intrinsics.principalPoint.x);
  output.mat[1][1] = static_cast<ARdouble>(input.intrinsics.focalLength.y);
  // Flip y since ARToolkit uses y-down.
  output.mat[1][2] =
    static_cast<ARdouble>(input.resolution.y) -
    static_cast<ARdouble>(input.intrinsics.principalPoint.y);

  return output;
}

// Convert the result of a pose estimate (from, e.g.,
// arGetTransMatMultiSquare(), into a Matrix4f representing a camera from world
// transformation with y-up.
Matrix4f ConvertToCameraFromWorldMeters(ARdouble matrix[3][4]) {
  double view_matrix[16];
  arglCameraViewRH(matrix, view_matrix, 0.001);  // scale factor: mm --> m.

  Matrix4f output;
  for (int k = 0; k < 16; ++k) {
    output[k] = static_cast<float>(view_matrix[k]);
  }
  const Matrix4f rot90 = Matrix4f::rotateX(static_cast<float>(M_PI_2));
  return output * rot90;
}

}

ARToolkitPoseEstimator::ARToolkitPoseEstimator(
  const CameraParameters& params, const std::string& marker_cube_filename) :
  luma_(params.resolution) {
  // Setup camera.
  ARParam camera_params = Convert(params);
  camera_params_lut_ =
    arParamLTCreate(&camera_params, AR_PARAM_LT_DEFAULT_OFFSET);
  tracker_ = arCreateHandle(camera_params_lut_);
  arSetPixelFormat(tracker_, pixel_format_);
  arSetMatrixCodeType(tracker_, AR_MATRIX_CODE_3x3_HAMMING63);

#if DEBUG
  arSetDebugMode(tracker_, AR_DEBUG_ENABLE);
#else
  arSetDebugMode(tracker_, AR_DEBUG_DISABLE);
#endif
  tracker_3d_ = ar3DCreateHandle(&camera_params);

  // Setup cube marker.
  pattern_ = arPattCreateHandle();

  // TODO: handle failure here.
  multi_marker_config_ = arMultiReadConfigFile(marker_cube_filename.c_str(),
    pattern_);
  // Yuck: ARToolkit doesn't set this rather important flag.
  multi_marker_config_->min_submarker = 1;

  // Set the detector type based on the marker type.
  const int patt_type = multi_marker_config_->patt_type;
  if (patt_type == AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE) {
    arSetPatternDetectionMode(tracker_, AR_TEMPLATE_MATCHING_COLOR);
  } else if (patt_type == AR_MULTI_PATTERN_DETECTION_MODE_MATRIX) {
    arSetPatternDetectionMode(tracker_, AR_MATRIX_CODE_DETECTION);
  } else { // AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE_AND_MATRIX
    arSetPatternDetectionMode(tracker_, AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX);
  }

  // Attach pattern to tracker.
  int ret = arPattAttach(tracker_, pattern_);
  assert(ret == 0);
}

ARToolkitPoseEstimator::~ARToolkitPoseEstimator() {
  if (tracker_3d_ != nullptr) {
    ar3DDeleteHandle(&tracker_3d_);
  }
  if (tracker_ != nullptr) {
    arDeleteHandle(tracker_);
  }
  if (camera_params_lut_ != nullptr) {
    arParamLTFree(&camera_params_lut_);
  }
}

#include "libcgt/core/common/ArrayUtils.h"
#include "libcgt/core/time/TimeUtils.h"

ARToolkitPoseEstimator::Result ARToolkitPoseEstimator::EstimatePose(
  Array2DReadView<uint8x3> bgr) {
  Result result;
  if (!bgr.packed()) {
    return result;
  }

  // Ugh need to convert anyway!.

  AR2VideoBufferT frame = {};
  // TODO: move into libcgt::colormath, make map() work.
  // TODO: fixed point to luma
  libcgt::core::arrayutils::map(bgr, luma_.writeView(),
    [&] (uint8x3 bgr)
    {
      float z = 0.25f * bgr.x + 0.5f * bgr.y + 0.25f * bgr.z;
      return static_cast<uint8_t>(z);
    } );

  frame.buff = reinterpret_cast<ARUint8*>(const_cast<uint8x3*>(bgr.pointer()));
  frame.buffLuma = luma_.pointer();
  frame.fillFlag = 1;
  // TODO: timestamps

  auto t0 = std::chrono::high_resolution_clock::now();

  int detect_result = arDetectMarker(tracker_, &frame);
  if (detect_result == 0) {
    ARMarkerInfo* detected_markers = arGetMarker(tracker_);
    result.nMarkers = arGetMarkerNum(tracker_);
    if (result.nMarkers > 0) {
      // TODO: add a config option for this.
      //ARdouble err = arGetTransMatMultiSquare(tracker_3d_, detected_markers,
      ARdouble err = arGetTransMatMultiSquareRobust(tracker_3d_, detected_markers,
        result.nMarkers, multi_marker_config_);
      if (err != -1.0) {
        Matrix4f trans = ConvertToCameraFromWorldMeters(
          multi_marker_config_->trans);
        result.error = err;
        result.world_from_camera = inverse(
          EuclideanTransform::fromMatrix(trans));
        result.valid = true;
      }
    }
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  if(result.valid) {
    printf("ARToolkit: succeeded, nMarkers = %d, error = %lf.\n",
      result.nMarkers, result.error);
  } else {
    printf("ARToolkit: failed.\n");
  }
  printf("ARToolkit: pose estimation took %lld ms\n",
    libcgt::core::time::dtMS(t0, t1));
  return result;
}
