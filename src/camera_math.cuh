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
#ifndef CAMERA_MATH_CUH
#define CAMERA_MATH_CUH

#include <cmath>
#include <vector_functions.h>

#include <cuda/float4x4.h>

// Convert a camera-space point into a pixel coordinate.
// The camera-space point is right handed with, x-right, y-up, z out of screen.
// The output pixel has x-right, y-up, and z equal to the -camera_point.z
// (a depth value that is positive into the screen).
__inline__ __device__ __host__
float3 PixelFromCamera(float3 camera_point, float4 flpp) {
  float depth = -camera_point.z;
  return{
    flpp.x * camera_point.x / depth + flpp.z,
    flpp.y * camera_point.y / depth + flpp.w,
    depth
  };
}

// Convert a world-space point into a pixel coordinate.
// The output pixel has x-right, y-up, and z equal to the -camera_point.z
// (a depth value that is positive into the screen).
__inline__ __device__ __host__
float3 PixelFromWorld(float3 world_point, float4x4 camera_from_world,
  float4 flpp) {
  float3 camera_point = transformPoint(camera_from_world, world_point);
  return PixelFromCamera(camera_point, flpp);
}

// Convert a pixel coordinate to an camera-space point given depth.
// The camera-space point is right handed with, x-right, y-up, z out of screen.
// xy takes on integer values at pixel corners and half-integer values at pixel
// centers. depth is a camera-space orthogonal depth and positive into the
// screen.
__inline__ __device__ __host__
float3 CameraFromPixel(float2 xy, float depth, float4 flpp) {
  return{
    depth * (xy.x - flpp.z) / flpp.x,
    depth * (xy.y - flpp.w) / flpp.y,
    -depth
  };
}

// Convert a pixel coordinate to an camera-space point given depth.
// The camera-space point is right handed with, x-right, y-up, z out of screen.
// xy is a pixel subscript and (0.5, 0.5) will be added to make it a pixel
// center. depth is a camera-space orthogonal depth and positive into the
// screen.
__inline__ __device__ __host__
float3 CameraFromPixel(int2 xy, float depth, float4 flpp) {
  return CameraFromPixel(
    make_float2(xy.x + 0.5f, xy.y + 0.5f),
    depth, flpp
  );
}

// Convert a pixel coordinate to a camera-space unit direction.
// xy takes on integer values at pixel corners and half-integer values at pixel
// centers.
__inline__ __device__ __host__
float3 CameraDirectionFromPixel(float2 xy, float4 flpp) {
  float fx = flpp.x;
  float fy = flpp.y;
  float dx = xy.x - flpp.z;
  float dy = xy.y - flpp.w;
  float den = sqrt(dx * dx * fy * fy + dy * dy * fx * fx + fx * fx * fy * fy);
  return{
    dx * fy / den,
    dy * fx / den,
    -fx * fy / den
  };
}

// Convert a pixel coordinate to a camera-space unit direction.
// xy is a pixel subscript and (0.5, 0.5) will be added to make it a pixel
// center.
__inline__ __device__ __host__
float3 CameraDirectionFromPixel(int2 xy, float4 flpp) {
  return CameraDirectionFromPixel(
    make_float2(xy.x + 0.5f, xy.y + 0.5f),
    flpp
  );
}

// Convert a pixel coordinate to an world-space point given depth.
// xy takes on integer values at pixel corners and half-integer values at pixel
// centers. depth is a camera-space orthogonal depth.
__inline__ __device__ __host__
float3 WorldFromPixel(float2 xy, float depth,
  float4 flpp, float4x4 world_from_camera) {
  return transformPoint(world_from_camera, CameraFromPixel(xy, depth, flpp));
}

// Convert a pixel coordinate to an world-space point given depth.
// xy is a pixel subscript and (0.5, 0.5) will be added to make it a pixel
// center. depth is a camera-space orthogonal depth.
__inline__ __device__ __host__
float3 WorldFromPixel(int2 xy, float depth,
  float4 flpp, float4x4 world_from_camera) {
  return transformPoint(world_from_camera, CameraFromPixel(xy, depth, flpp));
}

// Convert a pixel to a world-space unit direction.
// xy takes on integer values at pixel corners and half-integer values at pixel
// centers.
__inline__ __device__ __host__
float3 WorldDirectionFromPixel(float2 xy, float4 flpp,
  float4x4 world_from_camera) {
  float3 d = CameraDirectionFromPixel(xy, flpp);
  return transformVector(world_from_camera, d);
}

// Convert a pixel to a world-space unit direction.
// xy is a pixel subscript and (0.5, 0.5) will be added to make it a pixel
// center.
__inline__ __device__ __host__
float3 WorldDirectionFromPixel(int2 xy, float4 flpp,
  float4x4 world_from_camera) {
  float3 d = CameraDirectionFromPixel(xy, flpp);
  return transformVector(world_from_camera, d);
}

#endif // CAMERA_MATH_CUH
