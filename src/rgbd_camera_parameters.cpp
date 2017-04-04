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
#include "rgbd_camera_parameters.h"

#include "libcgt/core/io/PortableFloatMapIO.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"
#include "libcgt/opencv_interop/Calib3d.h"
#include "libcgt/opencv_interop/VecmathUtils.h"
#include <third_party/pystring/pystring.h>

using namespace pystring;
using namespace libcgt::core::vecmath;

using libcgt::opencv_interop::cameraMatrixToIntrinsics;

namespace {

CameraParameters LoadCameraIntrinsics(const cv::FileStorage& fs,
	const std::string& namePrefix) {
	CameraParameters params = {};

	cv::Size image_size;
	cv::Mat intrinsics_matrix_gl;
	cv::Mat dist_coeffs;
  cv::Mat undistorted_intrinsics_matrix_gl;

	fs[namePrefix + "ImageSize"] >> image_size;
	fs[namePrefix + "CameraMatrix_gl"] >> intrinsics_matrix_gl;
	fs[namePrefix + "DistCoeffs"] >> dist_coeffs;
  fs[namePrefix + "NewCameraMatrix_gl" ] >> undistorted_intrinsics_matrix_gl;

	params.resolution = { image_size.width, image_size.height };

  params.intrinsics = cameraMatrixToIntrinsics(intrinsics_matrix_gl);
	for (int i = 0; i < 5; ++i) {
		params.dist_coeffs.push_back(
			static_cast<float>(dist_coeffs.at<double>(0, i)));
	}

  params.undistorted_intrinsics = cameraMatrixToIntrinsics(
    undistorted_intrinsics_matrix_gl);

	return params;
}

}  // namespace

EuclideanTransform LoadEuclideanTransform(const cv::FileStorage& fs,
	const std::string& namePrefix) {
	cv::Mat r;
	cv::Mat t;

	fs[namePrefix + "_R"] >> r;
	fs[namePrefix + "_T"] >> t;

	return EuclideanTransform
	{
		libcgt::opencv_interop::fromCV3x3(r),
		libcgt::opencv_interop::fromCV3x1(t)
	};
}

// TODO: clean up how we load calibration data. Allow passing in a
// precomputed undistortion map.
bool LoadCameraParameters(const std::string& yaml,
  CameraParameters* params) {
  cv::FileStorage fs(yaml, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return false;
  }

  *params = {};

	cv::Size image_size;
	cv::Mat intrinsics_matrix_gl;
	cv::Mat dist_coeffs;
  cv::Mat undistorted_intrinsics_matrix_gl;

	fs["imageSize"] >> image_size;
	fs["cameraMatrix_gl"] >> intrinsics_matrix_gl;
	fs["distCoeffs"] >> dist_coeffs;
  fs["newCameraMatrix_gl" ] >> undistorted_intrinsics_matrix_gl;

	params->resolution = { image_size.width, image_size.height };

  params->intrinsics = cameraMatrixToIntrinsics(intrinsics_matrix_gl);
	for (int i = 0; i < 5; ++i) {
		params->dist_coeffs.push_back(
			static_cast<float>(dist_coeffs.at<double>(0, i)));
	}

  params->undistorted_intrinsics = cameraMatrixToIntrinsics(
    undistorted_intrinsics_matrix_gl);

#if 0
  auto res = PortableFloatMapIO::read(undistort_map_gl_filename);
  if (!res.valid) {
    return false;
  }
	params->undistortion_map = res.rg;
#endif

	return params;
}

bool LoadRGBDCameraParameters(const std::string& dir,
  RGBDCameraParameters* params) {
	cv::FileStorage fs(
		os::path::join(dir, "stereo_calibration.yaml"),
		cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return false;
  }

	params->color = LoadCameraIntrinsics(fs, "color");
  auto res = PortableFloatMapIO::read(
    os::path::join(dir, "color_undistort_map_gl.pfm2"));
  if (!res.valid) {
    return false;
  }
	params->color.undistortion_map = res.rg;

	params->depth = LoadCameraIntrinsics(fs, "depth");
  res = PortableFloatMapIO::read(
			os::path::join(dir, "depth_undistort_map_gl.pfm2"));
  if (!res.valid) {
    return false;
  }
	params->depth.undistortion_map = res.rg;

	params->color_from_depth = LoadEuclideanTransform(fs, "colorFromDepth_gl");
	params->depth_from_color = LoadEuclideanTransform(fs, "depthFromColor_gl");

	// TODO: depth calibration should output a range instead of the hard coded
  // constants here.
	params->depth.depth_range = Range1f::fromMinMax(0.8f, 4.0f);
	params->color.depth_range = params->depth.depth_range;

  return true;
}

EuclideanTransform RGBDCameraParameters::ConvertToColorCameraFromWorld(
    const EuclideanTransform& depth_camera_from_world) const {
  return color_from_depth * depth_camera_from_world;
}

EuclideanTransform RGBDCameraParameters::ConvertToDepthCameraFromWorld(
    const EuclideanTransform& color_camera_from_world) const {
  return depth_from_color * color_camera_from_world;
}
