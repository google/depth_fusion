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

#include <gflags/gflags.h>

#include <opencv2/calib3d.hpp>

#include "libcgt/core/common/ArrayUtils.h"
#include "libcgt/core/time/TimeUtils.h"
#include "libcgt/opencv_interop/ArrayUtils.h"
#include "libcgt/opencv_interop/VecmathUtils.h"
#include "libcgt/opencv_interop/Calib3d.h"

#include "src/rgbd_camera_parameters.h"

using libcgt::core::arrayutils::copy;
using libcgt::core::arrayutils::flipY;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::opencv_interop::array2DViewAsCvMat;
using libcgt::opencv_interop::cvMatAsArray2DView;
using libcgt::opencv_interop::fromCV3x3;
using libcgt::opencv_interop::makeCameraMatrix;

DECLARE_bool(collect_perf);

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
ArucoPoseEstimator::ArucoPoseEstimator(const cv::aruco::Board& fiducial,
  const CameraParameters& params,
  const std::string& detector_params_filename) :
  board_(fiducial),
  rot_x_180_(Matrix4f::rotateX(static_cast<float>(M_PI))) {

  // TODO: write a utility function for this.
  camera_intrinsics_ = makeCameraMatrix(params.intrinsics.focalLength.x,
    params.intrinsics.principalPoint.x - 0.5f,
    params.resolution.y - (params.intrinsics.principalPoint.y - 0.5f));
  camera_dist_coeffs_ = cv::Mat::zeros(1,
    static_cast<int>(params.dist_coeffs.size()), CV_32F);
  for (int i = 0; i < static_cast<int>(params.dist_coeffs.size()); ++i) {
    camera_dist_coeffs_.at<float>(0, i) = params.dist_coeffs[i];
  }

  cv::Point3f p0 = fiducial.objPoints[0][0];
  cv::Point3f p1 = fiducial.objPoints[0][1];
  axis_length_meters_ = 2.0f * static_cast<float>(cv::norm(p1 - p0));

  // TODO: bake this as a static default option. And let them pass one in.
  bool read_ok = readDetectorParameters(detector_params_filename, detector_params_);
}

ArucoPoseEstimator::Result ArucoPoseEstimator::EstimatePose(
  Array2DReadView<uint8x3> input,
  Array2DWriteView<uint8x3> vis) const {

  std::chrono::high_resolution_clock::time_point t0;
  std::chrono::high_resolution_clock::time_point t1;

  if (FLAGS_collect_perf) {
    t0 = std::chrono::high_resolution_clock::now();
  }

  cv::Mat bgr_mat = array2DViewAsCvMat(input);
  ArucoPoseEstimator::Detection detection = Detect(bgr_mat);
  Refine(bgr_mat, &detection);
  ArucoPoseEstimator::Result result = EstimatePose(detection);

  if (FLAGS_collect_perf) {
    t1 = std::chrono::high_resolution_clock::now();
    printf("ArUco: pose estimation took %lld ms\n",
      libcgt::core::time::dtMS(t0, t1));
  }

  if (vis.notNull()) {
    copy(input, vis);
    VisualizeDetections(detection, true, vis);
    VisualizePoseEstimate(result, vis);
  }

  return result;
}

ArucoPoseEstimator::Detection
ArucoPoseEstimator::Detect(cv::Mat image) const {
  ArucoPoseEstimator::Detection result;
  cv::aruco::detectMarkers(image, board_.dictionary, result.corners,
    result.ids, detector_params_, result.rejected);
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

    // We only rotation by pi on one side and don't conjugate on both sides
    // because ArUco uses a coordinate system where objPoints have GL
    // conventions, but camera points are in CV conventions.
    // Therefore, what they compute is cv_camera <-- gl_board.
    EuclideanTransform camera_from_board = EuclideanTransform::fromMatrix(
      rot_x_180_ * rt);
    pose.world_from_camera = inverse(camera_from_board);
  }
  return pose;
}

// static
void ArucoPoseEstimator::VisualizeDetections(
  const ArucoPoseEstimator::Detection& detection,
  bool show_rejected, Array2DWriteView<uint8x3> output) {
  cv::Mat output_mat = array2DViewAsCvMat(output);
  if (detection.ids.size() > 0) {
      cv::aruco::drawDetectedMarkers(output_mat,
          detection.corners, detection.ids);
  }
  if (show_rejected && detection.rejected.size() > 0) {
      cv::aruco::drawDetectedMarkers(output_mat, detection.rejected,
          cv::noArray(), cv::Scalar(100, 0, 255));
  }
}

void ArucoPoseEstimator::VisualizePoseEstimate(
  const ArucoPoseEstimator::Result& estimate_result,
  Array2DWriteView<uint8x3> output) const {
  if (estimate_result.valid) {
    cv::Mat intrinsics(camera_intrinsics_);
    cv::Mat dist_coeffs(camera_dist_coeffs_);
    cv::Mat output_mat = array2DViewAsCvMat(output);
    cv::aruco::drawAxis(output_mat,
      intrinsics, dist_coeffs,
      estimate_result.camera_from_board_rotation,
      estimate_result.camera_from_board_translation,
      axis_length_meters_);
  }
}
