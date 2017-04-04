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
#include "libcgt/camera_wrappers/StreamConfig.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"
#include "libcgt/core/vecmath/SimilarityTransform.h"

#include "../input_buffer.h"
#include "../pose_utils.h"
#include "../regular_grid_fusion_pipeline.h"
#include "../rgbd_camera_parameters.h"
#include "../rgbd_input.h"

using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::SimilarityTransform;

// Inputs.
DEFINE_string(calibration_dir, "",
  "[Required] calibration directory for the RGBD camera.");
DEFINE_string(input_rgbd, "",
  "[Required] input .rgbd file.");
DEFINE_string(pose_estimator, "color_aruco_and_depth_icp",
  "[Required] Pose estimator. Valid options: \"color_aruco\", \"depth_icp\", "
  "\"color_aruco_and_depth_icp\", \"precomputed\" or "
  "\"precomputed_refine_with_depth_icp\"."
  "If \"precomputed\" or \"precomputed_refine_with_depth_icp\", "
  "--precomputed_pose is required.");
DEFINE_string(precomputed_pose, "",
  "[Optional] precomputed pose file.");

// Outputs.
DEFINE_string(output_mesh, "",
  "[Optional] If not-empty, save fused mesh as a .obj file.");
DEFINE_string(output_pose, "",
  "[Optional] If not-empty, save new pose estimates as a .pose file.");
DEFINE_string(output_tsdf3d, "",
  "[Optional] If non-empty, save the TSDF volume as a .tsdf3d file.");

// Options.
DEFINE_bool(collect_perf, false, "Collect performance statistics.");
DEFINE_bool(adaptive_raycast, true, "Use signed distance values themselves "
  "during raycasting rather than one voxel at a time. Much faster, slightly "
  "less accurate.");

// TODO: specify these as flags.
constexpr int kRegularGridResolution = 512;
constexpr float kRegularGridSideLength = 2.0f;
constexpr float kRegularGridVoxelSize =
  kRegularGridSideLength / kRegularGridResolution;

SimilarityTransform GetInitialWorldFromGrid() {
  // TODO: consider initializing the camera to be at the origin.
  if (FLAGS_pose_estimator == "depth_icp") {
    // Put the camera at the center of the front face of the cube.
    return SimilarityTransform(kRegularGridVoxelSize) *
           SimilarityTransform(Vector3f(-0.5f * kRegularGridResolution,
                                        -0.5f * kRegularGridResolution,
                                        -kRegularGridResolution));
  } else {
    // Put the origin at the bottom in y, centered in x and z.
    return SimilarityTransform(Vector3f(-0.5f * kRegularGridSideLength,
                                        0.0f,
                                        -0.5f * kRegularGridSideLength)) *
           SimilarityTransform(kRegularGridVoxelSize);
  }
}

bool GetPoseEstimatorOptions(const RGBDCameraParameters& camera_params,
  PoseEstimatorOptions* options) {
  if (FLAGS_pose_estimator == "color_aruco" ||
    FLAGS_pose_estimator == "color_aruco_and_depth_icp") {
    if (FLAGS_pose_estimator == "color_aruco") {
      options->method = PoseEstimationMethod::COLOR_ARUCO;
    } else {
      options->method = PoseEstimationMethod::COLOR_ARUCO_AND_DEPTH_ICP;
    }
    return true;
  } else if (FLAGS_pose_estimator == "depth_icp") {
    // y up
    const EuclideanTransform kInitialDepthCameraFromWorld =
      EuclideanTransform::fromMatrix(
        Matrix4f::lookAt(
          { 0, 0, camera_params.depth.depth_range.minimum() },
          Vector3f{ 0 },
          Vector3f{ 0, 1, 0 }.normalized()
        )
      );

    options->method = PoseEstimationMethod::DEPTH_ICP;
    options->initial_pose.depth_camera_from_world =
      kInitialDepthCameraFromWorld;
    options->initial_pose.color_camera_from_world =
      camera_params.ConvertToColorCameraFromWorld(
        kInitialDepthCameraFromWorld);

    return true;
  } else if (FLAGS_pose_estimator == "precomputed" ||
    FLAGS_pose_estimator == "precomputed_refine_with_depth_icp") {
    if (FLAGS_pose_estimator == "precomputed") {
      options->method = PoseEstimationMethod::PRECOMPUTED;
    } else {
      options->method =
        PoseEstimationMethod::PRECOMPUTED_REFINE_WITH_DEPTH_ICP;
    }
    options->precomputed_path = LoadPoseHistory(FLAGS_precomputed_pose,
      camera_params.depth_from_color);
    if (options->precomputed_path.size() == 0) {
      fprintf(stderr, "Error: failed to load precomputed poses from %s\n",
        FLAGS_precomputed_pose.c_str());
      return false;
    }

    return true;
  } else {
    fprintf(stderr, "Invalid pose estimator: %s.\n",
      FLAGS_pose_estimator.c_str());
    return false;
  }
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // If no outputs, return immediately.
  if (FLAGS_output_mesh == "" &&
    FLAGS_output_pose == "" &&
    FLAGS_output_tsdf3d == "") {
    fprintf(stderr, "No outputs specified, returning immediately.\n");
    return 1;
  }

  bool ok;

  RGBDCameraParameters camera_params;
  ok = LoadRGBDCameraParameters(FLAGS_calibration_dir, &camera_params);
  if (!ok) {
    fprintf(stderr, "Error loading RGBD camera parameters from %s.\n",
      FLAGS_calibration_dir.c_str());
    return 2;
  }

  // TODO: validate rgbd input size with camera calibration size.
  // It may not have a color stream.
  RgbdInput rgbd_input(RgbdInput::InputType::FILE, FLAGS_input_rgbd.c_str());

  PoseEstimatorOptions pose_options;
  ok = GetPoseEstimatorOptions(camera_params, &pose_options);
  if (!ok) {
    fprintf(stderr, "Failed to parse pose estimator options.");
    return 2;
  }
  fprintf(stderr, "Using pose estimator: %s\n", FLAGS_pose_estimator.c_str());

  RegularGridFusionPipeline pipeline(camera_params,
    Vector3i(kRegularGridResolution),
    GetInitialWorldFromGrid(),
    pose_options);

  bool color_updated;
  bool depth_updated;
  rgbd_input.read(&(pipeline.GetInputBuffer()),
                  &color_updated,
                  &depth_updated);
  while (color_updated || depth_updated) {
    if (color_updated) {
      pipeline.NotifyColorUpdated();
    } else if (depth_updated) {
      pipeline.NotifyDepthUpdated();
    }
    rgbd_input.read(&(pipeline.GetInputBuffer()),
                    &color_updated,
                    &depth_updated);
  }

  // Fusion finished, save outputs.
  if (FLAGS_output_mesh != "") {
    TriangleMesh mesh = pipeline.Triangulate();
    fprintf(stderr, "Saving mesh to %s...", FLAGS_output_mesh.c_str());
    ok = mesh.saveOBJ(FLAGS_output_mesh);
    if (ok) {
      fprintf(stderr, "done.\n");
    } else {
      fprintf(stderr, "FAILED.\n");
    }
  }

  if (FLAGS_output_pose != "") {
    printf("Saving poses to %s...", FLAGS_output_pose.c_str());
    ok = SavePoseHistory(pipeline.PoseHistory(), FLAGS_output_pose);
    if (ok) {
      fprintf(stderr, "done.\n");
    } else {
      fprintf(stderr, "FAILED.\n");
    }
  }

  if (FLAGS_output_tsdf3d != "") {
    printf("Saving TSDF volume to %s...", FLAGS_output_tsdf3d.c_str());
    ok = pipeline.SaveTSDF3D(FLAGS_output_tsdf3d);
    if (ok) {
      fprintf(stderr, "done.\n");
    } else {
      fprintf(stderr, "FAILED.\n");
    }
  }
}
