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
#ifndef POSE_UTILS_H
#define POSE_UTILS_H

#include <string>
#include <vector>

#include "libcgt/core/vecmath/EuclideanTransform.h"

#include "pose_frame.h"

std::vector<PoseFrame> LoadPoseHistory(const std::string& filename,
  const libcgt::core::vecmath::EuclideanTransform& depth_from_color);

bool SavePoseHistory(const std::vector<PoseFrame>& pose_history,
  const std::string& filename);

#endif  // POSE_UTILS_H
