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

layout(location = 0) uniform sampler2D uSampler;
// Initialize to identity, scalar constructor is along *diagonal*.
layout(location = 1) uniform mat4 uColorMatrix = mat4(1.0);

in VertexData
{
    layout(location = 0) in vec2 vTex;
};

layout(location = 0) out vec4 outputColor;

void main()
{
    vec4 inputColor = texture(uSampler, vTex);
    outputColor = uColorMatrix * inputColor;
}
)"