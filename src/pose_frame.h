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
#ifndef POSE_FRAME_H
#define POSE_FRAME_H

#include <cstdint>

#include <core/vecmath/EuclideanTransform.h>

struct PoseFrame {

  using EuclideanTransform = libcgt::core::vecmath::EuclideanTransform;

  int frame_index;
  int64_t timestamp;
  EuclideanTransform camera_from_world;
};

#endif  // POSE_FRAME_H
