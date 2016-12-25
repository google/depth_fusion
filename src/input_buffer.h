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
#ifndef INPUT_BUFFER_H
#define INPUT_BUFFER_H

#include <mutex>

#include "libcgt/core/common/Array2D.h"
#include "libcgt/core/common/BasicTypes.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"
#include "libcgt/core/vecmath/Vector2i.h"

struct InputBuffer {

  InputBuffer(const Vector2i& color_resolution,
    const Vector2i& depth_resolution);

  // Incoming metadata.
  int color_frame_index = 0;
  int64_t color_timestamp_ns = 0LL;
  int depth_frame_index = 0;
  int64_t depth_timestamp_ns = 0LL;

  // This buffer is y-down for OpenCV only.
  Array2D<uint8x3> color_bgr_ydown;

  // These buffers are y-up for processing and GL.
  Array2D<uint8x3> color_rgb;
  Array2D<float> depth_meters; // depth in meters
};

#endif  // INPUT_BUFFER_H
