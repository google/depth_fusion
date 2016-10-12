R"(
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
#version 450

layout(location = 0) uniform mat4 uClipFromWorld;
// Initialize to identity, scalar constructor is along *diagonal*.
layout(location = 1) uniform mat4 uWorldFromObject = mat4(1.0);
layout(location = 2) uniform mat3 uWorldNormalFromObject = mat3(1.0);

layout(location = 0) in vec4 aPos;
layout(location = 1) in vec3 aNormal;

out gl_PerVertex
{
    vec4 gl_Position;
};

out VertexData
{
    layout(location = 0) out vec4 vWorldPos;
    layout(location = 1) out vec3 vWorldNormal;
};

void main()
{
  vWorldPos = uWorldFromObject * aPos;
  //vWorldNormal = uWorldNormalFromObject * aNormal;
  vWorldNormal = aNormal;

  gl_Position = uClipFromWorld * vWorldPos;
}
)"