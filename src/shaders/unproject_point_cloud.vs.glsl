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

layout(location = 0) uniform mat4 uClipFromWorld; // Viewing camera.

// Depth camera intrinsics.
layout(location = 1) uniform vec4 uDepthCameraFLPP;
layout(location = 2) uniform vec2 uDepthCameraRangeMinMax;
// Depth camera extrinsics.
layout(location = 3) uniform mat4 uDepthWorldFromCamera;

// Depth map texture.
layout(location = 4) uniform sampler2D uDepthSampler;

// Color of the point.
layout(location = 5) uniform vec4 uPointColor = vec4(1.0);

// Incoming position is a point.
layout(location = 0) in vec2 aPos;

out gl_PerVertex
{
    vec4 gl_Position;
};

out VertexData
{
    layout(location = 0) out vec4 vColor;
};

void main()
{
    // Unproject using focal length and principal point.
    // x = fx * X / Z + px
    // --> X = Z * (x - px) / fx.

    // Get the texture coordinates for this point.
    vec2 st = aPos / textureSize(uDepthSampler, 0);
    // Sample the depth texture to get a positive z.
    float z = texture2D(uDepthSampler, st).x;

    // If the sample is missing, output alpha = 0.
    // The fragment shader will discard it.
    float within_depth_range = 1;
    if (z < uDepthCameraRangeMinMax.x || z > uDepthCameraRangeMinMax.y)
    {
        within_depth_range = 0;
    }

    vec2 fl = uDepthCameraFLPP.xy;
    vec2 pp = uDepthCameraFLPP.zw;
    vec2 xy = z * (aPos - pp) / fl;

    // Now put it in depth camera coordinates.
    // Flip z coordinate for OpenGL and set w = 1.
    vec4 xyzw_depth_camera = vec4(xy, -z, 1);

    // Now transform it into world coordinates.
    vec4 xyzw_world = uDepthWorldFromCamera * xyzw_depth_camera;

    gl_Position = uClipFromWorld * xyzw_world;
    vColor = within_depth_range * uPointColor;
}
)"