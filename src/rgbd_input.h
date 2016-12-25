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
#ifndef RGBD_INPUT_H
#define RGBD_INPUT_H

#include <cstdint>
#include <memory>

#include "libcgt/core/common/Array2D.h"
#include "libcgt/core/common/BasicTypes.h"
#include "libcgt/camera_wrappers/RGBDStream.h"
#include "libcgt/camera_wrappers/OpenNI2/OpenNI2Camera.h"

// TODO(jiawen): Figure out a way to forward declare RGBDInputStream.

struct InputBuffer;

// TODO(jiawen): When accepting a camera, take in:
// - an array of StreamConfigs, which is generic.
// - a calibration file, or empty string for "use factory calibration."
class RgbdInput {
public:

  enum class InputType {
    KINECT1X,
    OPENNI2,
    REALSENSE,
    FILE,
  };

  RgbdInput() = default;
  RgbdInput(InputType input_type, const char* filename);

  Vector2i colorSize() const;
  Vector2i depthSize() const;

  // Read one frame. If rgb_updated is set to true, then buffer->color_rgb
  // will be updated. Likewise, if depth_updated is set to true, then
  // buffer->depth_meters will be updated. Both might be set to false, in which
  // case the read failed.
  //
  // TODO: return a status struct, indicating if end of file is reached.
  void read(InputBuffer* buffer, bool* rgb_updated, bool* depth_updated);

private:

  using OpenNI2Camera = libcgt::camera_wrappers::openni2::OpenNI2Camera;
  using RGBDInputStream = libcgt::camera_wrappers::RGBDInputStream;
  using StreamMetadata = libcgt::camera_wrappers::StreamMetadata;

  InputType input_type_;

  std::unique_ptr<OpenNI2Camera> openni2_camera_;
  Array2D<uint8x3> openni2_buffer_rgb_;
  Array2D<uint16_t> openni2_buffer_depth_;
  OpenNI2Camera::FrameView openni2_frame_;

  std::unique_ptr<RGBDInputStream> file_input_stream_;

  int color_stream_id_ = -1;
  StreamMetadata color_metadata_;

  int raw_depth_stream_id_ = -1;
  StreamMetadata depth_metadata_;
};

#endif  // RGBD_INPUT_H
