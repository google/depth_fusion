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

  // TODO: pass in a cv::aruco::DetectorParameters object instead of a
  // filename.
  ArucoPoseEstimator(const cv::aruco::Board& fiducial,
    const CameraParameters& params,
    const std::string& detector_params_filename);

  // Estimate the camera pose given a color image input. Input should have
  // PixelFormat "bgr".
  //
  // [Optional] If vis is not null, also draws a visualization of the pose
  // estimate into vis. vis should have PixelFormat BGR.
  //
  // In both cases, the y axis points down (OpenCV convention).
  Result EstimatePose(Array2DReadView<uint8x3> input,
    Array2DWriteView<uint8x3> vis = Array2DWriteView<uint8x3>()) const;

private:

  struct Detection {
    std::vector< int > ids;
    std::vector< std::vector< cv::Point2f > > corners;
    std::vector< std::vector< cv::Point2f > > rejected;
  };

  Detection Detect(cv::Mat image) const;

  // Refine the markers returned by Detect().
  // image must be the same as the one used in Detect().
  // detection is modified in place.
  void Refine(cv::Mat image, Detection* detection) const;

  // Estimate the camera pose from detected and optionally refined markers.
  // The Detection may be invalid if there are not enough corners.
  Result EstimatePose(const Detection& detection) const;

  cv::Mat camera_intrinsics_;
  cv::Mat camera_dist_coeffs_;

  cv::aruco::Board board_;
  cv::aruco::DetectorParameters detector_params_;

  // Rotation matrix about the x-axis by 180 degrees.
  const Matrix4f rot_x_180_;

  // For visualization.
  float axis_length_meters_;

  static void VisualizeDetections(const Detection& detection,
      bool show_rejected, Array2DWriteView<uint8x3> output);

  void VisualizePoseEstimate(const Result& pose_estimate,
    Array2DWriteView<uint8x3> output) const;
};

bool ReadDetectorParameters(const std::string& filename,
  cv::aruco::DetectorParameters& params);

#endif  // ARUCO_POSE_ESTIMATOR_H
