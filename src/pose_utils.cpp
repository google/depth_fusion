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
#include "pose_utils.h"

#include "libcgt/camera_wrappers/PoseStream.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"

using libcgt::camera_wrappers::PoseInputStream;
using libcgt::camera_wrappers::PoseOutputStream;
using libcgt::camera_wrappers::PoseStreamFormat;
using libcgt::camera_wrappers::PoseStreamMetadata;
using libcgt::camera_wrappers::PoseStreamTransformDirection;
using libcgt::camera_wrappers::PoseStreamUnits;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::transformPoint;

std::vector<PoseFrame> LoadPoseHistory(const std::string& filename,
  const EuclideanTransform& depth_from_color) {
  std::vector<PoseFrame> output;

  // TODO: read other encodings properly, read the metadata, etc.
  PoseInputStream input_stream(filename.c_str());
  if (input_stream.isValid()) {
    PoseFrame p;
    p.method = PoseFrame::EstimationMethod::PRECOMPUTED;
    while (input_stream.read(p.frame_index, p.timestamp_ns,
      p.color_camera_from_world.rotation,
      p.color_camera_from_world.translation)) {
      p.depth_camera_from_world = depth_from_color * p.color_camera_from_world;
      output.push_back(p);
    }
  }

  return output;
}

bool SavePoseHistory(const std::vector<PoseFrame>& poses,
  const std::string& filename) {
  PoseStreamMetadata metadata;
  metadata.format = PoseStreamFormat::ROTATION_MATRIX_3X3_COL_MAJOR_AND_TRANSLATION_VECTOR_FLOAT;
  metadata.units = PoseStreamUnits::METERS;
  metadata.direction = PoseStreamTransformDirection::CAMERA_FROM_WORLD;

  PoseOutputStream output_stream(metadata, filename.c_str());

  for( size_t i = 0; i < poses.size(); ++i )
  {
    bool succeeded = output_stream.write(
      poses[i].frame_index, poses[ i ].timestamp_ns,
      poses[i].color_camera_from_world.rotation,
      poses[i].color_camera_from_world.translation
    );
    if( !succeeded ) {
      return false;
    }
  }
  return true;
}
