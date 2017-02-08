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
#include "regular_grid_tsdf.h"

#include <cassert>

#include <gflags/gflags.h>

#include "libcgt/cuda/Event.h"
#include "libcgt/cuda/MathUtils.h"
#include "libcgt/cuda/VecmathConversions.h"

#include "fuse.h"
#include "marching_cubes.h"
#include "raycast.h"

using libcgt::core::vecmath::SimilarityTransform;
using libcgt::core::vecmath::inverse;
using libcgt::cuda::Event;

DECLARE_bool(collect_perf);

// VoxelSize() = world_from_grid_.scale.
RegularGridTSDF::RegularGridTSDF(const Vector3i& resolution,
  const SimilarityTransform& world_from_grid) :
  RegularGridTSDF(resolution, world_from_grid, 4 * world_from_grid.scale) {
}

RegularGridTSDF::RegularGridTSDF(const Vector3i& resolution,
  const SimilarityTransform& world_from_grid, float max_tsdf_value) :
  device_grid_(resolution),
  bounding_box_(resolution),
  world_from_grid_(world_from_grid),
  grid_from_world_(inverse(world_from_grid)),
  max_tsdf_value_(max_tsdf_value) {
  assert(VoxelSize() > 0);
  assert(max_tsdf_value > 0);

  Reset();
}

void RegularGridTSDF::Reset() {
  TSDF empty(0, 0, max_tsdf_value_);
  device_grid_.fill(empty);
}

const SimilarityTransform& RegularGridTSDF::GridFromWorld() const {
  return grid_from_world_;
}

const SimilarityTransform& RegularGridTSDF::WorldFromGrid() const {
  return world_from_grid_;
}

Box3f RegularGridTSDF::BoundingBox() const {
  return bounding_box_;
}

Vector3i RegularGridTSDF::Resolution() const {
  return device_grid_.size();
}

float RegularGridTSDF::VoxelSize() const {
  return world_from_grid_.scale;
}

Vector3f RegularGridTSDF::SideLengths() const {
  return VoxelSize() * Resolution();
}

void RegularGridTSDF::Fuse(const Vector4f& depth_camera_flpp,
  const Range1f& depth_range,
  const Matrix4f& camera_from_world,
  const DeviceArray2D<float>& depth_data) {

  dim3 block_dim(16, 16, 1);
  dim3 grid_dim = libcgt::cuda::math::numBins2D(
    { device_grid_.width(), device_grid_.height() },
    block_dim
  );

  // TODO: move these into class or use Performance Collector class.
  static float msTotal = 0.0f;
  static int nIterationsTotal = 0;
  Event e;

  if (FLAGS_collect_perf) {
    e.recordStart();
  }

  FuseKernel<<<grid_dim, block_dim>>>(
    make_float4x4(world_from_grid_.asMatrix()),
    max_tsdf_value_,
    make_float4(depth_camera_flpp),
    make_float2(depth_range.left(), depth_range.right()),
    make_float4x4(camera_from_world),
    depth_data.readView(),
    device_grid_.writeView());

  if (FLAGS_collect_perf) {
    float msElapsed = e.recordStopSyncAndGetMillisecondsElapsed();

    msTotal += msElapsed;
    ++nIterationsTotal;

    printf("Fuse() took: %f ms\n", msElapsed);

    printf("%d average: %f\n", nIterationsTotal, msTotal / nIterationsTotal);
    printf("3x average: %f\n", 3.0f * msTotal / nIterationsTotal);
  }
}

void RegularGridTSDF::FuseMultiple(
  const std::vector<CalibratedPosedDepthCamera>& depth_cameras,
  const std::vector<DeviceArray2D<float>>& depth_maps) {
  dim3 block_dim(16, 16, 1);
  dim3 grid_dim = libcgt::cuda::math::numBins2D(
    { device_grid_.width(), device_grid_.height() },
    block_dim
  );

  // TODO: move these into class or use Performance Collector class.
  static float msTotal = 0.0f;
  static int nIterationsTotal = 0;
  Event e;

  if (FLAGS_collect_perf) {
    e.recordStart();
  }

  FuseMultipleKernel<<<grid_dim, block_dim>>>(
    make_float4x4(world_from_grid_.asMatrix()),
    max_tsdf_value_,
    depth_cameras[0],
    depth_cameras[1],
    depth_cameras[2],
    depth_maps[0].readView(),
    depth_maps[1].readView(),
    depth_maps[2].readView(),
    device_grid_.writeView());

  if (FLAGS_collect_perf) {
    float msElapsed = e.recordStopSyncAndGetMillisecondsElapsed();

    msTotal += msElapsed;
    ++nIterationsTotal;

    printf("FuseMultiple() took: %f ms, %d-run average: %f\n",
      msElapsed, nIterationsTotal, msTotal / nIterationsTotal);
  }
}

void RegularGridTSDF::AdaptiveRaycast(const Vector4f& depth_camera_flpp,
  const Matrix4f& world_from_camera,
  DeviceArray2D<float4>& world_points_out,
  DeviceArray2D<float4>& world_normals_out) {
  dim3 block_dim(16, 16, 1);
  dim3 grid_dim = libcgt::cuda::math::numBins2D(
    { world_points_out.width(), world_points_out.height() },
    block_dim
  );

  Vector4f eye = world_from_camera * Vector4f(0, 0, 0, 1);
  float voxels_per_meter = 1.0f / VoxelSize();

  static float msTotal = 0.0f;
  static int nIterationsTotal = 0;
  Event e;

  if (FLAGS_collect_perf) {
    e.recordStart();
  }

  AdaptiveRaycastKernel<<<grid_dim, block_dim>>>(
    device_grid_.readView(),
    make_float4x4(grid_from_world_.asMatrix()),
    make_float4x4(world_from_grid_.asMatrix()),
    max_tsdf_value_,
    voxels_per_meter,
    make_float4(depth_camera_flpp),
    make_float4x4(world_from_camera),
    make_float3(eye.xyz),
    world_points_out.writeView(),
    world_normals_out.writeView()
  );

  if (FLAGS_collect_perf) {
    float msElapsed = e.recordStopSyncAndGetMillisecondsElapsed();

    msTotal += msElapsed;
    ++nIterationsTotal;

    printf("AdaptiveRaycastKernel took: %f ms\n", msElapsed);

    printf("%d run average: %f ms\n",
      nIterationsTotal, msTotal / nIterationsTotal);
    printf("resolution: %d x %d\n", world_points_out.width(),
      world_points_out.height());
  }
}

void RegularGridTSDF::Raycast(const Vector4f& depth_camera_flpp,
  const Matrix4f& world_from_camera,
  DeviceArray2D<float4>& world_points_out,
  DeviceArray2D<float4>& world_normals_out) {

  dim3 block_dim(16, 16, 1);
  dim3 grid_dim = libcgt::cuda::math::numBins2D(
    { world_points_out.width(), world_points_out.height() },
    block_dim
  );

  Vector4f eye = world_from_camera * Vector4f(0, 0, 0, 1);

  static float msTotal = 0.0f;
  static int nIterationsTotal = 0;
  Event e;

  if (FLAGS_collect_perf) {
    e.recordStart();
  }

  RaycastKernel<<<grid_dim, block_dim>>>(
    device_grid_.readView(),
    make_float4x4(grid_from_world_.asMatrix()),
    make_float4x4(world_from_grid_.asMatrix()),
    max_tsdf_value_,
    make_float4(depth_camera_flpp),
    make_float4x4(world_from_camera),
    make_float3(eye.xyz),
    world_points_out.writeView(),
    world_normals_out.writeView()
  );

  if (FLAGS_collect_perf) {
    float msElapsed = e.recordStopSyncAndGetMillisecondsElapsed();

    msTotal += msElapsed;
    ++nIterationsTotal;

    printf("RaycastKernel took: %f ms\n", msElapsed);

    printf("%d run average: %f ms\n",
      nIterationsTotal, msTotal / nIterationsTotal);
    printf("resolution: %d x %d\n", world_points_out.width(),
      world_points_out.height());
  }
}

TriangleMesh RegularGridTSDF::Triangulate() const {
  Array3D<TSDF> host_grid(Resolution());
  copy(device_grid_, host_grid.writeView());

  std::vector<Vector3f> positions;
  std::vector<Vector3f> normals;
  MarchingCubes(host_grid, max_tsdf_value_, world_from_grid_,
    positions, normals);

  return ConstructMarchingCubesMesh(positions, normals);
}
