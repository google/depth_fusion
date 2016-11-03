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
#include "projective_point_plane_icp.h"

#include <chrono>

#include <helper_math.h>

#include <thrust/execution_policy.h>
#include <thrust/reduce.h>

#include <core/vecmath/Quat4f.h>
#include <core/time/TimeUtils.h>
#include <cuda/Event.h>
#include <cuda/MathUtils.h>
#include <cuda/Rect2i.h>
#include <cuda/ThreadMath.cuh>
#include <cuda/VecmathConversions.h>

#include "camera_math.cuh"

using libcgt::cuda::contains;
using libcgt::cuda::math::floorToInt;
using libcgt::cuda::Rect2i;
using libcgt::cuda::threadmath::threadSubscript2DGlobal;
using libcgt::core::vecmath::EuclideanTransform;

// TODO(jiawen): can potentially optimize this.
struct Plus {
  __host__ __device__
    ICPLeastSquaresData operator () (const ICPLeastSquaresData& lhs,
      const ICPLeastSquaresData& rhs) const {
    ICPLeastSquaresData sum;
    for (int i = 0; i < 21; ++i) {
      sum.a[i] = lhs.a[i] + rhs.a[i];
    }
    for (int i = 0; i < 6; ++i) {
      sum.b[i] = lhs.b[i] + rhs.b[i];
    }
    sum.num_samples = lhs.num_samples + rhs.num_samples;
    sum.squared_residual = lhs.squared_residual + rhs.squared_residual;
    return sum;
  }
};

// TODO(jiawen): make a version without debug output
__global__
void ICPKernel(
  float4 flpp, // depth camera intrinsics
  float2 depth_min_max,
  float4x4 model_from_world,   // known model pose
  float4x4 model_from_current, // current pose estimate
  float4x4 current_from_model, // current pose estimate
  KernelArray2D<const float> depth_map,
  KernelArray2D<const float4> normal_map,
  KernelArray2D<const float4> world_points,
  KernelArray2D<const float4> world_normals,
  int src_image_guard_band_pixels,
  float max_distance_for_match,
  float min_dot_product_for_match,
  KernelArray2D<ICPLeastSquaresData> icp_data_out,
  KernelArray2D<uchar4> debug_vis_out) {
  // TODO: weighting function parameters
  // Reject data association if position differs by more than eps1
  // and normal dot product more than eps2?

  int2 dst_xy = threadSubscript2DGlobal();
  ICPLeastSquaresData output = {};
  uchar4 debug_output = {};

  float4 dst_point_world4 = world_points[dst_xy];
  float4 dst_normal_world4 = world_normals[dst_xy];

  if (dst_point_world4.w == 0 || dst_normal_world4.w == 0) {
    icp_data_out[dst_xy] = output;
    debug_vis_out[dst_xy] = debug_output;
    return;
  }

  float3 dst_point_world = make_float3(dst_point_world4);
  float3 dst_normal_world = make_float3(dst_normal_world4);

  float3 dst_point_model = transformPoint(model_from_world, dst_point_world);
  float3 dst_normal_model = transformVector(model_from_world, dst_normal_world);

  // Project dst_point into current pose estimate to see if it associates.
  float3 dst_point_current = transformPoint(current_from_model, dst_point_model);
  int2 dst_xy_current = floorToInt(make_float2(PixelFromCamera(
    dst_point_current, flpp)));

  Rect2i valid_rect = inset(Rect2i(depth_map.size()),
    src_image_guard_band_pixels);
  // If the point is in front of the camera, then dst_point_current.z < 0.
  if (!contains(valid_rect, dst_xy_current) || dst_point_current.z > 0) {
    icp_data_out[dst_xy] = output;
    debug_vis_out[dst_xy] = uchar4{ 255, 0, 0, 255 };
    return;
  }

  float src_depth = depth_map[dst_xy_current];
  float4 src_normal_current4 = normal_map[dst_xy_current];

  if (src_depth < depth_min_max.x || src_depth > depth_min_max.y ||
    src_normal_current4.w == 0) {
    icp_data_out[dst_xy] = output;
    debug_vis_out[dst_xy] = uchar4{ 0, 255, 0, 255 };
    return;
  }

  // Unproject src pixel into camera coordinates and then into model camera
  // coordinates.
  float3 src_point_current = CameraFromPixel(dst_xy_current, src_depth, flpp);
  float3 src_point_model = transformPoint(model_from_current,
    src_point_current);
  float3 src_normal_model = transformVector(model_from_current,
    make_float3(src_normal_current4));

  // TODO(jiawen): write a parameterized weight function which accepts points,
  // including 0.
  float3 delta = dst_point_model - src_point_model;
  if (length(delta) > max_distance_for_match) {
    icp_data_out[dst_xy] = output;
    debug_vis_out[dst_xy] = uchar4{ 0, 0, 255, 255 };
    return;
  }

  if (dot(src_normal_model, dst_normal_model) < min_dot_product_for_match) {
    icp_data_out[dst_xy] = output;
    debug_vis_out[dst_xy] = uchar4{ 255, 255, 0, 255 };
    return;
  }

  // Declare sample as valid.
  output.num_samples = 1;

  float3 c = cross(src_point_model, dst_normal_model);
  float r = dot(delta, dst_normal_model);

  output.a[ 0] = c.x * c.x;
  output.a[ 1] = c.y * c.x;
  output.a[ 2] = c.z * c.x;
  output.a[ 3] = dst_normal_model.x * c.x;
  output.a[ 4] = dst_normal_model.y * c.x;
  output.a[ 5] = dst_normal_model.z * c.x;

  output.a[ 6] = c.y * c.y;
  output.a[ 7] = c.z * c.y;
  output.a[ 8] = dst_normal_model.x * c.y;
  output.a[ 9] = dst_normal_model.y * c.y;
  output.a[10] = dst_normal_model.z * c.y;

  output.a[11] = c.z * c.z;
  output.a[12] = dst_normal_model.x * c.z;
  output.a[13] = dst_normal_model.y * c.z;
  output.a[14] = dst_normal_model.z * c.z;

  output.a[15] = dst_normal_model.x * dst_normal_model.x;
  output.a[16] = dst_normal_model.y * dst_normal_model.x;
  output.a[17] = dst_normal_model.z * dst_normal_model.x;

  output.a[18] = dst_normal_model.y * dst_normal_model.y;
  output.a[19] = dst_normal_model.z * dst_normal_model.y;

  output.a[20] = dst_normal_model.z * dst_normal_model.z;

  output.b[0] = c.x * r;
  output.b[1] = c.y * r;
  output.b[2] = c.z * r;
  output.b[3] = dst_normal_model.x * r;
  output.b[4] = dst_normal_model.y * r;
  output.b[5] = dst_normal_model.z * r;

  output.squared_residual = r * r;

  icp_data_out[dst_xy] = output;
  debug_vis_out[dst_xy] = uchar4{ 255, 255, 255, 255 };
}

Matrix4f rigidTransformationFromApprox(float x[6]) {
  float alpha = x[0];
  float beta  = x[1];
  float gamma = x[2];

  // TODO(jiawen): investigate using this one instead, it should work too and
  // is more symmetric.
#if 0
  return Matrix4f
  (
    1.0f,   -gamma,   beta,   x[3],
    gamma,    1.0f, -alpha,   x[4],
    -beta,   alpha,   1.0f,   x[5],
    0.0f,     0.0f,   0.0f,   1.0f
  );
#else
  return Matrix4f::translation({ x[3], x[4], x[5] }) *
    Matrix4f::rotateZ(gamma) *
    Matrix4f::rotateY(beta) *
    Matrix4f::rotateX(alpha);
#endif
}

ProjectivePointPlaneICP::ProjectivePointPlaneICP(
  const Vector2i& depth_resolution,
  const Intrinsics& depth_intrinsics, const Range1f& depth_range) :
  icp_data_(depth_resolution),
  depth_intrinsics_flpp_{ depth_intrinsics.focalLength,
    depth_intrinsics.principalPoint },
  depth_range_(depth_range) {
}

// TODO(jiawen): can improve conditioning by subtracting off the mean first
// This would make c = p x n smaller.

// Estimate camera pose using ICP.
__host__
ProjectivePointPlaneICP::Result ProjectivePointPlaneICP::EstimatePose(
  DeviceArray2D<float>& incoming_depth,
  DeviceArray2D<float4>& incoming_normals,
  const EuclideanTransform& world_from_camera,
  DeviceArray2D<float4>& world_points,
  DeviceArray2D<float4>& world_normals,
  DeviceArray2D<uchar4>& debug_vis) {
  auto t0 = std::chrono::high_resolution_clock::now();

  dim3 block_dim(16, 16, 1);
  dim3 grid_dim = libcgt::cuda::math::numBins2D(
    { incoming_depth.width(), incoming_depth.height() },
    block_dim
  );

  const ICPLeastSquaresData zero = {};

  ProjectivePointPlaneICP::Result result;

  const float4x4 model_from_world = make_float4x4(
    inverse(world_from_camera).asMatrix());
  Matrix4f model_from_current = Matrix4f::identity();
  for (int i = 0; i < kNumIterations; ++i) {
    Matrix4f current_from_model =
      Matrix4f::inverseEuclidean(model_from_current);

    ICPKernel<<<grid_dim, block_dim>>>(
      make_float4(depth_intrinsics_flpp_),
      make_float2(depth_range_.leftRight()),
      model_from_world,
      make_float4x4(model_from_current),
      make_float4x4(current_from_model),
      incoming_depth.readView(),
      incoming_normals.readView(),
      world_points.readView(),
      world_normals.readView(),
      kImageGuardBand,
      kMaxDistanceForMatch,
      kMinDotProductForMatch,
      icp_data_.writeView(),
      debug_vis.writeView());

    ICPLeastSquaresData* begin = icp_data_.pointer();
    ICPLeastSquaresData* end = icp_data_.rowPointer(icp_data_.height());
    // TODO(jiawen): benchmark how long the sum takes. Look into if the sum
    // can be computed on the GPU as well as the inversion.
    ICPLeastSquaresData sum =
      thrust::reduce(thrust::device, begin, end, zero, Plus());

    printf("ICP iteration %d (after assoc, before solve)\n"
      "num_samples = %d, squared_residual = %f\n",
      i, sum.num_samples, sum.squared_residual);

    if (sum.num_samples < kMinNumSamples) {
      result.valid = false;
      result.num_samples = sum.num_samples;
      return result;
    }

    // TODO(jiawen): benchmark how long the solve takes.
    float x[6];
    Solve(sum, x);
    Matrix4f incremental = rigidTransformationFromApprox(x);
    model_from_current = incremental * model_from_current;
  }

  Quat4f q = Quat4f::fromRotationMatrix(model_from_current.getSubmatrix3x3());
  Vector3f t = model_from_current.getCol(3).xyz;
  float radians;
  Vector3f axis = q.getAxisAngle(&radians);

  if (t.norm() > kMaxTranslation || radians > kMaxRotationRadians) {
    result.valid = false;
    return result;
  }

  result.valid = true;

  Matrix4f new_world_from_camera = world_from_camera.asMatrix() * model_from_current;
  Vector4f eye = new_world_from_camera * Vector4f(0, 0, 0, 1);
  Vector4f center = new_world_from_camera * Vector4f(0, 0, -1, 1);
  Vector4f up = new_world_from_camera * Vector4f(0, 1, 0, 0);

  Matrix4f camera_from_world = Matrix4f::lookAt(eye.xyz, center.xyz, up.xyz);
  result.world_from_camera = EuclideanTransform::fromMatrix(
    Matrix4f::inverseEuclidean(camera_from_world));

  auto t1 = std::chrono::high_resolution_clock::now();
  printf("ICP took %lld ms\n", libcgt::core::time::dtMS(t0, t1));

  return result;
}
