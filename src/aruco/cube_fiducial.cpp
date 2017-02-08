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
#include "cube_fiducial.h"

#include "libcgt/core/common/Array2D.h"
#include "libcgt/core/common/ArrayUtils.h"
#include "libcgt/core/geometry/BoxUtils.h"
#include "libcgt/core/vecmath/Vector2i.h"
#include "libcgt/opencv_interop/ArrayUtils.h"
#include "libcgt/opencv_interop/VecmathUtils.h"

using cv::aruco::Board;
using cv::Point3f;
using libcgt::core::arrayutils::flipYInPlace;
using libcgt::core::geometry::corners;
using libcgt::core::geometry::makeBox;
using libcgt::core::geometry::solidBoxTriangleListIndices;
using libcgt::opencv_interop::array2DViewAsCvMat;
using libcgt::opencv_interop::toCVPoint;
using std::vector;

CubeFiducial::CubeFiducial(float side_length) :
  CubeFiducial(makeBox(Vector3f{0}, side_length)) {
}

CubeFiducial::CubeFiducial(const Box3f& box) :
  box_(box) {
  objPoints.resize(kNumMarkers, vector<Point3f>(kNumPointsPerMarker));
  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  ids = {0, 1, 2, 3, 4, 5};

  // Vertex positions in hypercube order.
  // The code box has the same center but is only 3/4 the size.
  Box3f code_box = makeBox(box.center(), 0.75f * box.size.x);
  Array1D<Vector4f> box_positions = corners(code_box);
  // Each face, looked head on, in clockwise order, from the top left.
  const int aruco_cube_face_indices[kNumMarkers][kNumPointsPerMarker] = {
    {2, 6, 4, 0}, // left
    {7, 3, 1, 5}, // right
    {4, 5, 1, 0}, // bottom
    {2, 3, 7, 6}, // top
    {3, 2, 0, 1}, // back
    {6, 7, 5, 4}  // front
  };

  for (int i = 0; i < kNumMarkers; ++i) {
    for (int j = 0; j < 4; ++j) {
      Vector3f p = box_positions[aruco_cube_face_indices[i][j]].xyz;
      objPoints[i][j] = toCVPoint(p);
    }
  }
}

CubeFiducialDrawable::CubeFiducialDrawable(const CubeFiducial& fiducial,
  int face_size_texels) :
  GLDrawable(GLPrimitiveType::TRIANGLES, calculator()),
  texture_({CubeFiducial::kNumMarkers * face_size_texels, face_size_texels},
    GLImageInternalFormat::R8) {
  // Vertex positions in hypercube order.
  Array1D<Vector4f> box_positions = corners(fiducial.box_);
  Array1D<int> gl_indices = solidBoxTriangleListIndices();

  // Populate GL positions.
  {
    auto mb = mapAttribute<Vector3f>(0);
    Array1DWriteView<Vector3f> positions = mb.view();
    for (size_t i = 0; i < gl_indices.size(); ++i) {
      positions[i] = box_positions[gl_indices[i]].xyz;
    }
  }

  // Populate GL texture coordinates.
  {
    float inv_face_size = 1.0f / CubeFiducial::kNumMarkers;
    auto mb = mapAttribute<Vector2f>(1);
    Array1DWriteView<Vector2f> texture_coordinates = mb.view();
    for (int i = 0; i < CubeFiducial::kNumMarkers; ++i) {
      int b = CubeFiducial::kNumMarkers * i;
      texture_coordinates[b    ] = Vector2f{i * inv_face_size      , 0.0f};
      texture_coordinates[b + 1] = Vector2f{(i + 1) * inv_face_size, 0.0f};
      texture_coordinates[b + 2] = Vector2f{i * inv_face_size      , 1.0f};

      texture_coordinates[b + 3] = Vector2f{i * inv_face_size      , 1.0f};
      texture_coordinates[b + 4] = Vector2f{(i + 1) * inv_face_size, 0.0f};
      texture_coordinates[b + 5] = Vector2f{(i + 1) * inv_face_size, 1.0f};
    }
  }

  // Set the texture itself.
  //
  // dict.markerSize = bits per dimension.
  // Add a 1 bit border on each side.
  constexpr int kBorderBits = 1;

  // Each face comprises an outer border that's white (a ring 1 bit wide),
  // a black border around the code (another ring 1 bit wide), followed by
  // the 4x4 code.

  // First, fill the texture with white.
  texture_.clear(static_cast<uint8_t>(255u));

  // For one face, the code with the inner border occupies 3/4 of the width and
  // height.
  // Since it's centered, it is offset 1/8 of the way from the corner.
  Vector2i code_with_border_size{face_size_texels * 3 / 4};
  Vector2i code_offset{face_size_texels / 8};
  Array2D<uint8_t> face_tex_data(code_with_border_size);
  cv::Mat cv_view = array2DViewAsCvMat(face_tex_data.writeView());

  for (int i = 0; i < CubeFiducial::kNumMarkers; ++i) {
    int id = fiducial.ids[i];
    // TODO: draw the marker directly using the dictionary byte list instead
    // of using dict.drawMarker, draws then resizes.
    fiducial.dictionary.drawMarker(id, code_with_border_size.x, cv_view,
      kBorderBits);
    flipYInPlace(face_tex_data.writeView());
    texture_.set(face_tex_data, GLImageFormat::RED,
      {face_size_texels * i + code_offset.x, code_offset.y} /* dstOffset */ );
  }

  texture_.setSwizzleRGBAlpha(GLTexture::SwizzleTarget::RED);
}

const GLTexture2D& CubeFiducialDrawable::texture() const {
  return texture_;
}

// static
PlanarVertexBufferCalculator CubeFiducialDrawable::calculator() {
  constexpr int kNumTriangles = 2 * CubeFiducial::kNumMarkers;
  constexpr int nVertices = 3 * kNumTriangles;
  PlanarVertexBufferCalculator calculator( nVertices );
  calculator.addAttribute< Vector3f >();
  calculator.addAttribute< Vector2f >();
  return calculator;
}
