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
#ifndef ARUCO_POSE_ESTIMATOR_H
#define ARUCO_POSE_ESTIMATOR_H

#include <cmath>

#include <opencv2/aruco.hpp>

#include "libcgt/core/common/Array2D.h"
#include "libcgt/core/common/BasicTypes.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"

struct CameraParameters;

class ArucoPoseEstimator {
 public:

  using EuclideanTransform = libcgt::core::vecmath::EuclideanTransform;

  struct Detection {
    std::vector< int > ids;
    std::vector< std::vector< cv::Point2f > > corners;
    std::vector< std::vector< cv::Point2f > > rejected;
  };

  // Pose estimate.
  //
  // If valid = true:
  //
  // camera_from_board will be a rigid transform mapping camera <-- board.
  //
  // camera_from_board_rotation and camera_from_board_translation represents
  // the same transformation but as as rotation and translation vectors.
  //
  // Coordinate conventions:
  //
  // Aruco stores the board's coordinates using OpenGL conventions.
  //
  // camera_from_board is a fully-OpenGL transformation, mapping a GL board
  // to a GL camera (y up, z out of screen for both).
  //
  // camera_from_board_rotation and camera_from_board_translation maps from
  // a GL board (y up, z out of screen) to a CV camera
  // (y down, z into screen).
  //
  // Therefore, there is a 180 degree rotation about the x axis between the
  // two representations.
  struct Result {
    bool valid = false;

    EuclideanTransform world_from_camera;

    cv::Vec3d camera_from_board_rotation;
    cv::Vec3d camera_from_board_translation;
  };

  // TODO(jiawen): also pass in a cv::aruco::DetectorParameters object.
  ArucoPoseEstimator(const CameraParameters& params,
      const std::string& detector_params_filename);

  // Generate the board image. width and height are in pixels.
  // Returns a cv::Mat(height, width, CV_8U).
  cv::Mat BoardImage(int width, int height);

  // Generate the board image, flipped up/down as a Array2D<uint8_t> so that
  // it is suitable for OpenGL.
  // "width" and "height" are in pixels.
  Array2D<uint8_t> GLBoardImage(int width, int height);

  Detection Detect(cv::Mat image) const;

  // Refine the markers returned by Detect().
  // image must be the same as the one used in Detect().
  // detection is modified in place.
  void Refine(cv::Mat image, Detection* detection) const;

  // Estimate the camera pose from detected and optionally refined markers.
  // The Detection may be invalid if there are not enough corners.
  Result EstimatePose(const Detection& detection) const;

  Result EstimatePose(Array2DReadView<uint8x3> bgr) const;

  static void VisualizeDetections(const Detection& detection,
      bool show_rejected, cv::Mat* vis_image);

  void VisualizePoseEstimate(const Result& pose_estimate,
      cv::Mat* vis_image);

private:

  cv::Mat camera_intrinsics_;
  cv::Mat camera_dist_coeffs_;

  cv::aruco::Dictionary dictionary_;
  cv::aruco::GridBoard board_;
  cv::aruco::DetectorParameters detector_params_;

  // Number of markers in x and y.
  int markers_x_;
  int markers_y_;

  // Side length of one marker, in meters.
  float marker_length_;

  // Length of the empty space between markers:
  // the end of one marker and the start of the next.
  float marker_separation_;

  // Computed just for visualizing the axis.
  float axis_length_;

  // Rotation matrix about the x-axis by pi radians.
  const Matrix4f rot_x_pi_;
};

#endif  // ARUCO_POSE_ESTIMATOR_H
