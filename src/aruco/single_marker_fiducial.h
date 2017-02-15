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
#ifndef SINGLE_MARKER_FIDUCIAL_H
#define SINGLE_MARKER_FIDUCIAL_H

#include <opencv2/aruco.hpp>

#include "libcgt/core/vecmath/Rect2f.h"
#include "libcgt/GL/GL_45/drawables/Drawable.h"
#include "libcgt/GL/GL_45/GLTexture2D.h"

// A single fiducial marker. It is a subclass of cv::aruco::Board, which
// represents the data needed for any kind of 3D fiducial, not just planar
// ones.
class SingleMarkerFiducial : public cv::aruco::Board {
 public:

  static constexpr int kNumMarkers = 1;
  static constexpr int kNumPointsPerMarker = 4;
  static constexpr float kDefaultSideLength = 0.1524f;  // 6 inches, 152.4 mm.

  SingleMarkerFiducial(float side_length = kDefaultSideLength, int id = 0);

  // TODO: oriented rect
  const Rect2f rect_;
};

// TODO: implement me.
#if 0
// Must be instantiated when an OpenGL context is active.
class SingleMarkerFiducialDrawable : public GLDrawable {
 public:

  static constexpr int kDefaultFaceSizeTexels = 128;

  SingleMarkerFiducialDrawable(const SingleMarkerFiducial& fiducial,
    int face_size_texels = kDefaultFaceSizeTexels);

  const GLTexture2D& texture() const;

 private:

  static PlanarVertexBufferCalculator calculator();
  GLTexture2D texture_;
};
#endif

#endif  // SINGLE_MARKER_FIDUCIAL_H
