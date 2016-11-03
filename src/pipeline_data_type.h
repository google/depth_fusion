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
#ifndef PIPELINE_DATA_TYPE_H
#define PIPELINE_DATA_TYPE_H

#include <bitmask_operators.hpp>

enum class PipelineDataType {
  NONE = 0,
  INPUT_COLOR = 1,
  INPUT_DEPTH = 2,
  SMOOTHED_DEPTH = 4,
  CAMERA_POSE = 8,
  TSDF = 16,
  RAYCAST_NORMALS = 32
};

template<>
struct enable_bitmask_operators<PipelineDataType> {
  static const bool enable = true;
};

#endif  // PIPELINE_DATA_TYPE_H
