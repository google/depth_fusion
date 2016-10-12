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

// TODO(jiawen): rename these.
layout(location = 0) uniform mat4 uCameraFromWorld;
layout(location = 1) uniform vec4 uDepthCameraFLPP;
layout(location = 2) uniform vec2 uDepthCameraRangeMinMax;
layout(location = 3) uniform mat4 uDepthWorldFromCamera;
layout(location = 4) uniform mat4 uColorClipFromWorld;

layout(location = 5) uniform sampler2D uDepthSampler;
layout(location = 6) uniform sampler2D uColorSampler;

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
    // x = f * X / Z + cx
    // --> X = Z * (x - cx) / f.

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

    // Transform the point into color camera coordinates.
    vec4 xyzw_color_clip = uColorClipFromWorld * xyzw_world;
    vec2 st_color = xyzw_color_clip.xy / xyzw_color_clip.w;
    st_color = 0.5 * (st_color + vec2(1));

    vec4 output_color;
    if (st_color.x >= 0 && st_color.y >= 0 &&
        st_color.x <= 1 && st_color.y <= 1 &&
        xyzw_color_clip.w > 0)
    {
        // TODO(jiawen): use texture2DProj.
        // TODO(jiawen): use sampler objects correctly.
        output_color = within_depth_range * texture2D(uColorSampler, st_color);
    }
    else
    {
        // If the point is visible but has no corresponding color pixel,
        // color it red.
        output_color = within_depth_range * vec4(1, 0, 0, 1);
    }

    gl_Position = uCameraFromWorld * xyzw_world;
    vColor = output_color;
}
)"