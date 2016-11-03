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

#include <core/vecmath/EuclideanTransform.h>
#include <io/PortableFloatMapIO.h>
#include <opencv_interop/VecmathUtils.h>
#include <pystring/pystring.h>

using namespace pystring;
using namespace libcgt::core::vecmath;

CameraParameters LoadCameraIntrinsics(const cv::FileStorage& fs,
	const std::string& namePrefix) {
	CameraParameters params = {};

	cv::Size image_size;
	cv::Mat intrinsics_gl;
	cv::Mat dist_coeffs;

	fs[namePrefix + "ImageSize"] >> image_size;
	fs[namePrefix + "CameraMatrix_gl"] >> intrinsics_gl;
	fs[namePrefix + "DistCoeffs"] >> dist_coeffs;

	params.resolution = { image_size.width, image_size.height };
	params.intrinsics.focalLength.x =
		static_cast<float>(intrinsics_gl.at<double>(0, 0));
	params.intrinsics.focalLength.y =
		static_cast<float>(intrinsics_gl.at<double>(1, 1));
	params.intrinsics.principalPoint.x =
		static_cast<float>(intrinsics_gl.at<double>(0, 2));
	params.intrinsics.principalPoint.y =
		static_cast<float>(intrinsics_gl.at<double>(1, 2));
	for (int i = 0; i < 5; ++i) {
		params.dist_coeffs.push_back(
			static_cast<float>(dist_coeffs.at<double>(0, i)));
	}

	return params;
}

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

RGBDCameraParameters LoadRGBDCameraParameters(const std::string& dir) {
	RGBDCameraParameters params;
	cv::FileStorage fs(
		os::path::join(dir, "stereo_calibration.yaml"),
		cv::FileStorage::READ);

	params.color = LoadCameraIntrinsics(fs, "color");
	params.color.undistortion_map =
		PortableFloatMapIO::read(
			os::path::join(dir, "color_undistort_map_gl.pfm2")).rg;
	params.depth = LoadCameraIntrinsics(fs, "depth");
	params.depth.undistortion_map =
		PortableFloatMapIO::read(
			os::path::join(dir, "depth_undistort_map_gl.pfm2")).rg;

	params.color_from_depth = LoadEuclideanTransform(fs, "colorFromDepth_gl");
	params.depth_from_color = LoadEuclideanTransform(fs, "depthFromColor_gl");

	// TODO: depth calibration should output a range.
	params.depth.depth_range = Range1f::fromMinMax(0.8f, 4.0f);
	params.color.depth_range = params.depth.depth_range;

	return params;
}

EuclideanTransform RGBDCameraParameters::ConvertToColorCameraFromWorld(
    const EuclideanTransform& depth_camera_from_world) const {
  return color_from_depth * depth_camera_from_world;
}

EuclideanTransform RGBDCameraParameters::ConvertToDepthCameraFromWorld(
    const EuclideanTransform& color_camera_from_world) const {
  return depth_from_color * color_camera_from_world;
}
