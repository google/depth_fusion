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
#include <string>

#include <gflags/gflags.h>
#include "libcgt/camera_wrappers/PoseStream.h"
#include "libcgt/core/common/ArrayUtils.h"
#include "libcgt/core/common/StringUtils.h"
#include "libcgt/core/io/PortableFloatMapIO.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"
#include "third_party/pystring/pystring.h"

#include "../pose_frame.h"
#include "../pose_utils.h"
#include "../regular_grid_tsdf.h"
#include "../rgbd_camera_parameters.h"

// Options.
DEFINE_bool(collect_perf, false, "Collect performance statistics.");

// Inputs.
DEFINE_string(tsdf3d, "", "Input TSDF");
DEFINE_string(intrinsics, "", "Input camera intrinsics (.yaml)");
DEFINE_string(pose, "", "Input camera path (.pose)");

// Outputs.
// TODO: use undistorted intrinsics.
DEFINE_string(output_dir, "", "Output directory");
DEFINE_bool(output_depth, false, "Output depth"); // units are meters
DEFINE_bool(output_world_points, true, "Output world points");
DEFINE_bool(output_world_normals, true, "Output world normals");
// TODO: secondary bounce points and normals.
// TODO: adaptive raycast
// TODO: get rid of these

using libcgt::camera_wrappers::PoseInputStream;
using libcgt::camera_wrappers::PoseStreamFormat;
using libcgt::camera_wrappers::PoseStreamMetadata;
using libcgt::camera_wrappers::PoseStreamTransformDirection;
using libcgt::camera_wrappers::PoseStreamUnits;
using libcgt::core::cameras::Intrinsics;
using libcgt::core::arrayutils::cast;
using libcgt::core::arrayutils::flipY;
using libcgt::core::stringPrintf;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::SimilarityTransform;
using pystring::os::path::join;

struct TimestampedPose {
  int32_t frame_index;
  int64_t timestamp;
  EuclideanTransform camera_from_world;
};

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_output_dir.empty()) {
    fprintf(stderr, "output_dir is required.\n");
    return 1;
  }

  if (!FLAGS_output_depth &&
    !FLAGS_output_world_points &&
    !FLAGS_output_world_normals) {
    fprintf(stderr, "Must specify at least one output.\n");
    return 1;
  }

  if (FLAGS_output_depth) {
    fprintf(stderr, "Depth output not yet implemented.\n");
    return 1;
  }

  // Load intrinsics.
  CameraParameters camera_params;
  if (!LoadCameraParameters(FLAGS_intrinsics, &camera_params)) {
    fprintf(stderr, "Failed to read camera intrinsics.\n");
    return 1;
  }

  // TODO: implement load as a free function without a dummy constructor.
  // currently, we're stuck at 512^3!.
  Vector3i resolution{512};
  RegularGridTSDF tsdf(resolution, SimilarityTransform{});

  if (!tsdf.Load(FLAGS_tsdf3d)) {
    fprintf(stderr, "Error loading TSDF3D from %s\n.",
      FLAGS_tsdf3d.c_str());
    return 1;
  }

  std::vector<TimestampedPose> camera_path;

  int32_t frame_index;
  int64_t timestamp;
  EuclideanTransform e;

  // TODO: make a class and function for loading a camera path with intrinsics.
  PoseInputStream pose_stream(FLAGS_pose.c_str());
  bool ok = pose_stream.read(frame_index, timestamp,
    e.rotation, e.translation);
  while (ok) {
    if (pose_stream.metadata().direction ==
      PoseStreamTransformDirection::WORLD_FROM_CAMERA) {
      camera_path.push_back(
        TimestampedPose{frame_index, timestamp, inverse(e)});
    } else {
      camera_path.push_back(TimestampedPose{frame_index, timestamp, e});
    }
    ok = pose_stream.read(frame_index, timestamp, e.rotation, e.translation);
  }

  if (camera_path.empty()) {
    fprintf(stderr, "Error loading camera poses from %s\n.",
      FLAGS_pose.c_str());
  }

  DeviceArray2D<float4> world_points(camera_params.resolution);
  DeviceArray2D<float4> world_normals(camera_params.resolution);
  Array2D<Vector4f> host_world_points(camera_params.resolution);
  Array2D<Vector4f> host_world_normals(camera_params.resolution);

  const Vector4f flpp{camera_params.undistorted_intrinsics.focalLength,
    camera_params.undistorted_intrinsics.principalPoint};

  for (size_t i = 0; i < camera_path.size(); ++i) {
    printf("Raycasting frame %zu of %zu\n", i, camera_path.size());

    const auto& pose = camera_path[i];
    tsdf.Raycast(flpp, inverse(pose.camera_from_world).asMatrix(),
      world_points, world_normals);

    if (FLAGS_output_world_points || FLAGS_output_depth) {
      copy(world_points, cast<float4>(host_world_points.writeView()));

      std::string world_points_filename =
        stringPrintf("world_points_%05d_%020lld.pfm4",
          pose.frame_index, pose.timestamp);
      PortableFloatMapIO::write(flipY(host_world_points.readView()),
        join(FLAGS_output_dir, world_points_filename));
    }
    if (FLAGS_output_world_normals) {
      copy(world_normals, cast<float4>(host_world_normals.writeView()));

      std::string world_normals_filename =
        stringPrintf("world_normals_%05d_%020lld.pfm4",
          pose.frame_index, pose.timestamp);
      PortableFloatMapIO::write(flipY(host_world_normals.readView()),
        join(FLAGS_output_dir, world_normals_filename));
    }
  }

  return 0;
}
