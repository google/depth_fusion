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
#ifndef TSDF_H
#define TSDF_H

#include <vector_types.h>

class TSDF {
public:

  ushort2 encoded_ = ushort2{ 0, 0 };

  __inline__ __device__ __host__
  TSDF() = default;

  __inline__ __device__ __host__
  TSDF(float d, float w, float max_tsdf_value);

  __inline__ __device__ __host__
  TSDF(const TSDF& copy) = default;

  __inline__ __device__ __host__
  TSDF& operator = (const TSDF& copy) = default;

  __inline__ __device__ __host__
  float2 Get(float max_tsdf_value) const;

  __inline__ __device__ __host__
  float Distance(float max_tsdf_value) const;

  __inline__ __device__ __host__
  float Weight() const;

  __inline__ __device__ __host__
  void Set(float d, float w, float max_tsdf_value);

  __inline__ __device__ __host__
  void Update(float incoming_d, float incoming_w, float max_tsdf_value);
};

__inline__ __device__ __host__
TSDF::TSDF(float d, float w, float max_tsdf_value) {
  Set(d, w, max_tsdf_value);
}

__inline__ __device__ __host__
float2 TSDF::Get(float max_tsdf_value) const {
  return{ Distance(max_tsdf_value), Weight() };
}

__inline__ __device__ __host__
float TSDF::Distance(float max_tsdf_value) const {
  return (2 * max_tsdf_value * (encoded_.x / 65535.f)) - max_tsdf_value;
}

__inline__ __device__ __host__
float TSDF::Weight() const {
  return static_cast<float>(encoded_.y);
}

__inline__ __device__ __host__
void TSDF::Set(float d, float w, float max_tsdf_value) {
  encoded_ = {
    static_cast<unsigned short>((d + max_tsdf_value) / (2 * max_tsdf_value) * 65535),
    static_cast<unsigned short>(w)
  };
}

__inline__ __device__ __host__
void TSDF::Update(float incoming_d, float incoming_w, float max_tsdf_value) {
  float2 old_tsdf = Get(max_tsdf_value);
  float old_d = old_tsdf.x;
  float old_w = old_tsdf.y;

  float new_w = old_w + incoming_w;
  float new_d = (old_w * old_d + incoming_w * incoming_d) / new_w;

  Set(new_d, new_w, max_tsdf_value);
}

#endif  // TSDF_H
