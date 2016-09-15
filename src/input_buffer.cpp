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
#include "input_buffer.h"

InputBuffer::InputBuffer(const Vector2i& color_resolution,
  const Vector2i& depth_resolution) :
  color_bgr_ydown(color_resolution),
  color_rgb(color_resolution),
  depth_meters(depth_resolution) {

}
