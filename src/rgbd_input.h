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

#include <memory>

#include <camera_wrappers/RGBDStream.h>

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

  Vector2i rgbSize() const;
  Vector2i depthSize() const;

  void read(InputBuffer* state, bool* rgb_updated, bool* depth_updated);

private:

  using RGBDInputStream = libcgt::camera_wrappers::RGBDInputStream;

  std::unique_ptr<RGBDInputStream> file_input_stream_;

  int rgb_stream_id_ = -1;
  Vector2i rgb_size_;

  int raw_depth_stream_id_ = -1;
  Array2D<uint16_t> raw_depth_mm_;
};

#endif  // RGBD_INPUT_H
