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
#include "aruco_pose_estimator.h"

#include <chrono>

#include <opencv2/calib3d.hpp>

#include "libcgt/core/common/ArrayUtils.h"
#include "libcgt/core/time/TimeUtils.h"
#include "libcgt/opencv_interop/ArrayUtils.h"
#include "libcgt/opencv_interop/VecmathUtils.h"
#include "libcgt/opencv_interop/Calib3d.h"

#include "rgbd_camera_parameters.h"

using libcgt::core::arrayutils::copy;
using libcgt::core::arrayutils::flipY;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::opencv_interop::array2DViewAsCvMat;
using libcgt::opencv_interop::cvMatAsArray2DView;
using libcgt::opencv_interop::fromCV3x3;
using libcgt::opencv_interop::makeCameraMatrix;

bool readDetectorParameters(const std::string& filename, cv::aruco::DetectorParameters& params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }
    fs["adaptiveThreshWinSizeMin"] >> params.adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params.adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params.adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params.adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params.minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params.maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params.polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params.minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params.minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params.minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params.doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params.cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params.cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params.cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params.markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params.perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params.perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params.maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params.minOtsuStdDev;
    fs["errorCorrectionRate"] >> params.errorCorrectionRate;
    return true;
}

// 4x4_50: 4x4 bits (4 bits in each direction), 50 markers.
ArucoPoseEstimator::ArucoPoseEstimator(const CameraParameters& params,
  const std::string& detector_params_filename) :
  dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50)),
  markers_x_(2),
  markers_y_(2),
  marker_length_(0.064f),
  marker_separation_(0.064f),
  rot_x_pi_(Matrix4f::rotateX(static_cast<float>(M_PI))) {

  // TODO: write a utility function for this.
  camera_intrinsics_ = makeCameraMatrix(params.intrinsics.focalLength.x,
    params.intrinsics.principalPoint.x - 0.5f,
    params.resolution.y - (params.intrinsics.principalPoint.y - 0.5f));
  camera_dist_coeffs_ = cv::Mat::zeros(1, params.dist_coeffs.size(), CV_32F);
  for (int i = 0; i < static_cast<int>(params.dist_coeffs.size()); ++i) {
    camera_dist_coeffs_.at<float>(0, i) = params.dist_coeffs[i];
  }

  // Detailed documentation here:
  // http://docs.opencv.org/3.1.0/d9/d6d/tutorial_table_of_content_aruco.html#gsc.tab=0
  // Predefined dictionaries have "names" which are actually enum instances
  // of the form: DICT_<num_bits_x>X<num_bits_y>_<num_markers>
  // DICT_4X4_50 means: each marker is 4 bits by 4 bits. The dictionary
  // consists of 50 markers.

  board_ = cv::aruco::GridBoard::create(
      markers_x_, markers_y_,
      marker_length_, marker_separation_,
      dictionary_);

  axis_length_ = 0.5f * (marker_separation_ +
      (float)std::min(markers_x_, markers_y_) *
      (marker_length_ + marker_separation_));

  // TODO: bake this as a static default option. And let them pass one in.
  bool read_ok = readDetectorParameters(detector_params_filename, detector_params_);
}

cv::Mat ArucoPoseEstimator::BoardImage(int width, int height) {
    cv::Mat board_image(height, width, CV_8U);
    board_.draw(board_image.size(), board_image);
    return board_image;
}

Array2D<uint8_t> ArucoPoseEstimator::GLBoardImage(int width, int height) {
    Array2D<uint8_t> gl_board_image({width, height});
    cv::Mat cv_board_image = BoardImage(width, height);
    Array2DReadView<uint8_t> src = flipY(
      cvMatAsArray2DView<uint8_t>(cv_board_image));
    copy(src, gl_board_image.writeView());
    return gl_board_image;
}

ArucoPoseEstimator::Detection
ArucoPoseEstimator::Detect(cv::Mat image) const {
  ArucoPoseEstimator::Detection result;
  cv::aruco::detectMarkers(image, dictionary_, result.corners, result.ids,
    detector_params_, result.rejected);
  return result;
}

void ArucoPoseEstimator::Refine(cv::Mat image,
  ArucoPoseEstimator::Detection* result) const {
    cv::aruco::refineDetectedMarkers(image, board_,
      result->corners, result->ids, result->rejected,
      camera_intrinsics_, camera_dist_coeffs_);
}

ArucoPoseEstimator::Result ArucoPoseEstimator::EstimatePose(
  const Detection& detection) const {
  ArucoPoseEstimator::Result pose;
  pose.valid = false;

  int num_markers_detected = 0;
  if (detection.ids.size() > 0) {
    num_markers_detected =
      cv::aruco::estimatePoseBoard(
        detection.corners, detection.ids,
        board_,
        camera_intrinsics_, camera_dist_coeffs_,
        pose.camera_from_board_rotation,
        pose.camera_from_board_translation);
  }

  if (num_markers_detected > 0) {
    pose.valid = true;

    // Convert to GL.
    cv::Mat cv_rotation_matrix;
    cv::Rodrigues(pose.camera_from_board_rotation, cv_rotation_matrix);
    Matrix4f rt;
    rt.setSubmatrix3x3(0, 0, fromCV3x3(cv_rotation_matrix));
    rt(0, 3) = static_cast<float>(pose.camera_from_board_translation(0));
    rt(1, 3) = static_cast<float>(pose.camera_from_board_translation(1));
    rt(2, 3) = static_cast<float>(pose.camera_from_board_translation(2));
    rt(3, 3) = 1.0f;

    // TODO: y-up

    Matrix4f rotx90 = Matrix4f::rotateX(static_cast<float>(M_PI_2));

    // TODO: make a cube "board".

    // We only rotation by pi on one side and don't conjugate on both sides
    // because aruco's coordinate system is such that:
    // the board is in GL conventions, but the camera is in CV conventions.
    // Therefore, what they compute is cv_camera <-- gl_board.
    EuclideanTransform camera_from_board = EuclideanTransform::fromMatrix(
      rot_x_pi_ * rt * rotx90);
    pose.world_from_camera = inverse(camera_from_board);
  }
  return pose;
}

ArucoPoseEstimator::Result ArucoPoseEstimator::EstimatePose(
  Array2DReadView<uint8x3> bgr) const {
  //cv::setNumThreads(1);
  printf("Using %d threads\n", cv::getNumThreads());

  auto t0 = std::chrono::high_resolution_clock::now();

  cv::Mat bgr_mat = array2DViewAsCvMat(bgr);
  ArucoPoseEstimator::Detection detection = Detect(bgr_mat);

  auto t1 = std::chrono::high_resolution_clock::now();

  Refine(bgr_mat, &detection);

  auto t2 = std::chrono::high_resolution_clock::now();

  ArucoPoseEstimator::Result result = EstimatePose(detection);

  auto t3 = std::chrono::high_resolution_clock::now();

  // TODO: time is almost entirely in detect.
  // Using 48 threads, detection takes about 60 ms.
  // Using 1 thread, dtection takes about 230 ms.

  printf("ArUco: pose estimation took %lld ms\n",
    libcgt::core::time::dtMS(t0, t3));
  printf("ArUco: detect took %lld ms, refine took %lld ms, estimate pose took %lld ms\n",
    libcgt::core::time::dtMS(t0, t1),
    libcgt::core::time::dtMS(t1, t2),
    libcgt::core::time::dtMS(t2, t3));
  return result;
}

// static
void ArucoPoseEstimator::VisualizeDetections(
  const ArucoPoseEstimator::Detection& detection,
  bool show_rejected, cv::Mat* vis_image) {
  if (detection.ids.size() > 0) {
      cv::aruco::drawDetectedMarkers(*vis_image,
          detection.corners, detection.ids);
  }
  if (show_rejected && detection.rejected.size() > 0) {
      cv::aruco::drawDetectedMarkers(*vis_image, detection.rejected,
          cv::noArray(), cv::Scalar(100, 0, 255));
  }
}

void ArucoPoseEstimator::VisualizePoseEstimate(
  const ArucoPoseEstimator::Result& estimate_result,
  cv::Mat* vis_image) {
  if (estimate_result.valid) {
    cv::aruco::drawAxis(*vis_image,
      camera_intrinsics_, camera_dist_coeffs_,
      estimate_result.camera_from_board_rotation,
      estimate_result.camera_from_board_translation,
      axis_length_);
  }
}
