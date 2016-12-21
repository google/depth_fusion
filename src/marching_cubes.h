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
#ifndef MARCHING_CUBES_H
#define MARCHING_CUBES_H

#include <vector>

#include <core/common/Array3D.h>
#include <core/vecmath/SimilarityTransform.h>
#include <core/vecmath/Vector3f.h>
#include <core/geometry/TriangleMesh.h>

#include "tsdf.h"

// Run the marching cubes algorithm on the regular grid TSDF.
// Generates a triangle list of positions and normals.
void MarchingCubes(const Array3D< TSDF> grid, float max_tsdf_value,
  const libcgt::core::vecmath::SimilarityTransform& world_from_grid,
  std::vector<Vector3f>& triangle_list_positions_out,
  std::vector<Vector3f>& triangle_list_normals_out);

TriangleMesh ConstructMarchingCubesMesh(
  const std::vector<Vector3f>& triangle_list_positions);

TriangleMesh ConstructMarchingCubesMesh(
  const std::vector<Vector3f>& triangle_list_positions,
  const std::vector<Vector3f>& triangle_list_normals);

#endif  // MARCHING_CUBES_H
