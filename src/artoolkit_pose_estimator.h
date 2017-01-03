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
#ifndef AR_TOOLKIT_POSE_ESTIMATOR_H
#define AR_TOOLKIT_POSE_ESTIMATOR_H

#include <AR/arMulti.h>

#include "libcgt/core/common/Array2D.h"
#include "libcgt/core/common/BasicTypes.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"

struct CameraParameters;

class ARToolkitPoseEstimator {
 public:

  using EuclideanTransform = libcgt::core::vecmath::EuclideanTransform;

  struct Result {
    bool valid = false;
    int nMarkers;
    double error;
    EuclideanTransform world_from_camera;
  };

   ARToolkitPoseEstimator(const CameraParameters& params,
     const std::string& marker_cube_filename);
   ~ARToolkitPoseEstimator();

   Result EstimatePose(Array2DReadView<uint8x3> bgr);

 private:

  Array2D<uint8_t> luma_;

  // TODO: AR_PIXEL_FORMAT_MONO is probably good enough!
  AR_PIXEL_FORMAT pixel_format_ = AR_PIXEL_FORMAT_BGR;
  ARParamLT* camera_params_lut_ = nullptr;
  ARHandle* tracker_ = nullptr;
  AR3DHandle* tracker_3d_ = nullptr;

  ARPattHandle* pattern_ = nullptr;
  ARMultiMarkerInfoT* multi_marker_config_ = nullptr;
};

#endif  // AR_TOOLKIT_POSE_ESTIMATOR_H
