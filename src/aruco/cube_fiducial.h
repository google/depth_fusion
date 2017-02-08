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
#ifndef CUBE_FIDUCIAL_H
#define CUBE_FIDUCIAL_H

#include <opencv2/aruco.hpp>

#include "libcgt/core/vecmath/Box3f.h"
#include "libcgt/GL/GL_45/drawables/Drawable.h"
#include "libcgt/GL/GL_45/GLTexture2D.h"

// A cubical fiducial marker. It is a subclass of cv::aruco::Board, which
// represents the data needed for any kind of 3D fiducial, not just planar
// ones.
class CubeFiducial : public cv::aruco::Board {
 public:

  static constexpr int kNumMarkers = 6;
  static constexpr int kNumPointsPerMarker = 4;
  static constexpr float kDefaultSideLength = 0.064f;  // 64 mm.

  CubeFiducial(float side_length = kDefaultSideLength);
  CubeFiducial(const Box3f& box);

  // TODO: oriented box?
  const Box3f box_;
};

// Must be instantiated when an OpenGL context is active.
class CubeFiducialDrawable : public GLDrawable {
 public:

  static constexpr int kDefaultFaceSizeTexels = 128;


  CubeFiducialDrawable(const CubeFiducial& fiducial,
    int face_size_texels = kDefaultFaceSizeTexels);

  const GLTexture2D& texture() const;

 private:

  static PlanarVertexBufferCalculator calculator();
  GLTexture2D texture_;
};

#endif  // CUBE_FIDUCIAL_H
