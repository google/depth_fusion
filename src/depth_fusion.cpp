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
#include <cstdio>

#include <gflags/gflags.h>

#include <QApplication>

#include "libcgt/camera_wrappers/StreamConfig.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"
#include "libcgt/core/vecmath/SimilarityTransform.h"

#include "control_widget.h"
#include "input_buffer.h"
#include "main_widget.h"
#include "main_controller.h"
#include "pose_utils.h"
#include "regular_grid_fusion_pipeline.h"
#include "rgbd_camera_parameters.h"
#include "rgbd_input.h"

using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::SimilarityTransform;

DEFINE_bool(collect_perf, false, "Collect performance statistics.");
DEFINE_bool(adaptive_raycast, true, "Use signed distance values themselves "
  " during raycasting rather than one voxel at a time. Much faster, slightly "
  " less accurate.");
DEFINE_string(mode, "single_moving",
  "Mode to run the app in. Either \"single_moving\" or \"multi_static\"." );

// Single moving mode flags.
DEFINE_string(sm_calibration_dir, "",
  "REQUIRED for single moving mode: "
  "calibration directory for the RGBD camera.");
DEFINE_string(sm_input_type, "openni2",
  "REQUIRED for single moving mode: "
  "data source. Either \"openni2\" for the currently connected OpenNI2 "
  "camera, or a \"file\", in which case sm_input_args should specify the path "
  "to a .rgbd file.");
DEFINE_string(sm_input_args, "",
  "OPTIONAL for single moving mode: "
  "If sm_input_type is \"file\", the path to a .rgbd file.");
DEFINE_string(sm_pose_estimator, "color_aruco_and_depth_icp",
  "REQUIRED for single moving mode: "
  "Pose estimator. Valid options: \"color_aruco\", \"depth_icp\", "
  "\"color_aruco_and_depth_icp\", \"precomputed\" or "
  "\"precomputed_refine_with_depth_icp\"."
  "If \"precomputed\" or \"precomputed_refine_with_depth_icp\", "
  "input_pose is required.");
DEFINE_string(sm_pose_file, "",
  "Filename for precomputed pose path.");

// Multi static mode flags.
DEFINE_bool(ms_use_gui, true,
  "Set true to visualize with GUI, false to run in batch mode.");

int SingleMovingCameraMain(int argc, char* argv[]) {

  if (FLAGS_sm_calibration_dir == "") {
    printf("sm_calibration_dir is required.\n");
    return 1;
  }

  RgbdInput::InputType input_type;
  if (FLAGS_sm_input_type == "openni2") {
    input_type = RgbdInput::InputType::OPENNI2;
  } else if( FLAGS_sm_input_type == "file" ) {
    input_type = RgbdInput::InputType::FILE;
  } else {
    printf("sm_input_type must be \"openni2\" or \"file\"\n");
    return 1;
  }

  if (FLAGS_sm_input_type == "file") {
    if (FLAGS_sm_input_args == "") {
      printf("In FILE mode, sm_input_args is required.\n");
      return 1;
    }
  }

  QApplication app(argc, argv);

  RGBDCameraParameters camera_params;
  bool ok = LoadRGBDCameraParameters(FLAGS_sm_calibration_dir, &camera_params);
  if (!ok) {
    fprintf(stderr, "Error loading RGBD camera parameters from %s.\n",
      FLAGS_sm_calibration_dir);
    return 1;
  }

  const Vector3i kRegularGridResolution(512); // ~2m^3
  const float kRegularGridSideLength = 2.0f;
  const float kRegularGridVoxelSize =
    kRegularGridSideLength / kRegularGridResolution.x;

  RgbdInput rgbd_input(input_type, FLAGS_sm_input_args.c_str());

  std::unique_ptr<RegularGridFusionPipeline> pipeline;

  PoseEstimatorOptions pose_options;

  if (FLAGS_sm_pose_estimator == "color_aruco" ||
    FLAGS_sm_pose_estimator == "color_aruco_and_depth_icp") {
    // Put the origin at the center of the cube.
    const SimilarityTransform kInitialWorldFromGrid =
      SimilarityTransform(kRegularGridVoxelSize) *
      SimilarityTransform(Vector3f(-0.5f * kRegularGridResolution.x,
        -0.5f * kRegularGridResolution.y, -0.5f * kRegularGridResolution.z));

    if (FLAGS_sm_pose_estimator == "color_aruco") {
      pose_options.method = PoseEstimationMethod::COLOR_ARUCO;
    } else {
      pose_options.method = PoseEstimationMethod::COLOR_ARUCO_AND_DEPTH_ICP;
    }

    pipeline = std::make_unique<RegularGridFusionPipeline>(camera_params,
      kRegularGridResolution, kInitialWorldFromGrid, pose_options);
  } else if (FLAGS_sm_pose_estimator == "depth_icp") {

    // Put the camera at the center of the front face of the cube.
    const SimilarityTransform kInitialWorldFromGrid =
      SimilarityTransform(kRegularGridVoxelSize) *
      SimilarityTransform(Vector3f(-0.5f * kRegularGridResolution.x,
        -0.5f * kRegularGridResolution.y, -kRegularGridResolution.z));

    // y up
    const EuclideanTransform kInitialDepthCameraFromWorld =
      EuclideanTransform::fromMatrix(
        Matrix4f::lookAt(
          { 0, 0, camera_params.depth.depth_range.minimum() },
          Vector3f{ 0 },
          Vector3f{ 0, 1, 0 }.normalized()
        )
      );

    pose_options.method = PoseEstimationMethod::DEPTH_ICP;
    pose_options.initial_pose.depth_camera_from_world = kInitialDepthCameraFromWorld;
    pose_options.initial_pose.color_camera_from_world =
      camera_params.ConvertToColorCameraFromWorld(
        kInitialDepthCameraFromWorld);

    pipeline = std::make_unique<RegularGridFusionPipeline>(camera_params,
      kRegularGridResolution, kInitialWorldFromGrid,
      pose_options);
  } else if (FLAGS_sm_pose_estimator == "precomputed" ||
    FLAGS_sm_pose_estimator == "precomputed_refine_with_depth_icp") {
    // Put the origin at the bottom in y, centered in x and z.
    const SimilarityTransform kInitialWorldFromGrid =
      SimilarityTransform(Vector3f(-0.5f * kRegularGridSideLength, 0.0f, -0.5f * kRegularGridSideLength)) *
      SimilarityTransform(kRegularGridVoxelSize);

    if (FLAGS_sm_pose_estimator == "precomputed") {
      pose_options.method = PoseEstimationMethod::PRECOMPUTED;
    } else {
      pose_options.method = PoseEstimationMethod::PRECOMPUTED_REFINE_WITH_DEPTH_ICP;
    }
    pose_options.precomputed_path = LoadPoseHistory(FLAGS_sm_pose_file,
      camera_params.depth_from_color);
    if (pose_options.precomputed_path.size() == 0) {
      fprintf(stderr, "Error: failed to load pose history from %s\n",
        FLAGS_sm_pose_file.c_str());
      return 1;
    }

    pipeline = std::make_unique<RegularGridFusionPipeline>(camera_params,
      kRegularGridResolution, kInitialWorldFromGrid, pose_options);
  }

  ControlWidget control_widget;
  control_widget.setGeometry(50, 50, 150, 150);
  control_widget.show();

  MainWidget main_widget;
  main_widget.SetPipeline(pipeline.get());
  main_widget.show();

  const int window_width = 2240;
  const int window_height = 1200;
  int x = control_widget.geometry().right();
  int y = control_widget.geometry().top();
  main_widget.move(x, y);
  main_widget.resize(window_width, window_height);

  MainController controller(&rgbd_input, pipeline.get(),
    &control_widget, &main_widget);
  return app.exec();
}

#include "libcgt/core/vecmath/Quat4f.h"
#include "libcgt/opencv_interop/Calib3d.h"
#include "libcgt/opencv_interop/VecmathUtils.h"

using libcgt::opencv_interop::cameraMatrixCVToGL;
using libcgt::opencv_interop::cameraMatrixToIntrinsics;
using libcgt::opencv_interop::makeCameraMatrix;
using libcgt::opencv_interop::undistortRectifyMap;
using libcgt::opencv_interop::toCVSize;

// HACK: this only makes a depth camera at 512x424.
RGBDCameraParameters makeRGBDCameraParameters(double focalLength,
  double principalPointX, double principalPointY,
  double distortionR2, double distortionR4) {
  RGBDCameraParameters params;

  params.depth.resolution = { 512, 424 };
  params.depth.depth_range = Range1f::fromMinMax(0.1f, 10.0f);

  cv::Mat_<double> cvCameraMatrix = makeCameraMatrix(focalLength,
    principalPointX, principalPointY);
  cv::Size2i cvImageSize = toCVSize(params.depth.resolution);

  cv::Mat_<double> dist_coeffs = cv::Mat_<double>::zeros(1, 4);
  dist_coeffs(0, 0) = distortionR2;
  dist_coeffs(0, 1) = distortionR4;

  params.depth.dist_coeffs = std::vector<float>(4, 0.0f);
  params.depth.dist_coeffs[0] = static_cast<float>(distortionR2);
  params.depth.dist_coeffs[1] = static_cast<float>(distortionR4);

  params.depth.intrinsics = cameraMatrixToIntrinsics(
    cameraMatrixCVToGL(cvCameraMatrix, cvImageSize));

  params.depth.undistortion_map = undistortRectifyMap(
    cvCameraMatrix, dist_coeffs, cv::Mat(),
    cvCameraMatrix, cvImageSize);

  return params;
}

SimilarityTransform MakeWorldFromGrid(const Vector3f& center,
  float side_length, int resolution) {
  // Input grid coordinate p: [0, resolution]
  // Position relative to corner of the grid: q = p * voxel_size.
  // Position relative to the world: r = corner + q.
  Vector3f corner = center - Vector3f{0.5f * side_length};
  float voxel_size = side_length / resolution;

  auto s0 = SimilarityTransform(voxel_size);
  auto s1 = SimilarityTransform(corner);
  auto s2 = s1 * s0;

  auto t1 = SimilarityTransform(corner) * SimilarityTransform(voxel_size);

  return t1;
}

#include "libcgt/core/io/NumberedFilenameBuilder.h"


int MultiStaticCameraMain(int argc, char* argv[]) {
  QApplication app(argc, argv);

  std::vector<std::string> rgbd_stream_filenames = {
    "d:/tmp/multicam/kinect0083_new/filtered_frames.rgbd",
    "d:/tmp/multicam/kinect0134_new/filtered_frames.rgbd",
    "d:/tmp/multicam/kinect0142_new/filtered_frames.rgbd"
  };

  const int kNumCameras = 3;
  std::vector<RGBDCameraParameters> camera_params;
  camera_params.push_back(makeRGBDCameraParameters(365.183, 255.097, 207.695,
    0.0745901, -0.189055));
  camera_params.push_back(makeRGBDCameraParameters(366.009, 251.421, 205.132,
    0.0641811, -0.179726));
  camera_params.push_back(makeRGBDCameraParameters(365.712, 254.596, 205.857,
    0.0678888, -0.185925));

  std::vector<EuclideanTransform> camera_poses(3);
  camera_poses[0].translation = Vector3f{-0.015828f, 0.014736f, -0.0128686f};
  camera_poses[1].translation = Vector3f{1.77081f, -0.0100862f, 0.828459f};
  camera_poses[2].translation = Vector3f{0.934717f, 7.526e-05f, 0.2477f};

  Vector3f axisAngle;
  Quat4f q;

  axisAngle = Vector3f{-0.00690718f, -0.0101686f, -0.00351523f};
  camera_poses[0].rotation = Matrix3f::rotation(axisAngle);

  axisAngle = Vector3f{-0.0148537f, 0.919529f, -0.000168047f};
  camera_poses[1].rotation = Matrix3f::rotation(axisAngle);

  axisAngle = Vector3f{-0.0496128f, 0.417153f, -0.00555608f};
  camera_poses[2].rotation = Matrix3f::rotation(axisAngle);

  EuclideanTransform rot180(Matrix3f::rotateX(static_cast<float>(M_PI)));

  for(int i = 0; i < 3; ++i) {
    // Convert translation from SfM convention (R * (x[0:2] - x[3] * c)) to
    // standard convention: [R, t]: R * x[0:2] + t.
    camera_poses[i].translation =
      -camera_poses[i].rotation * camera_poses[i].translation;
    // Convert from y-down to y-up.
    camera_poses[i] = rot180 * camera_poses[i] * rot180;
  }

  // HACK: where to place grid.
  // Pick a pixel from camera 1.
  const Vector2f xy0{
    camera_params[1].depth.resolution.x / 2.0f,
    camera_params[1].depth.resolution.x / 4.0f};
  const float z0 = 2.5f;
  PerspectiveCamera center_camera(
    camera_poses[1],
    camera_params[1].depth.intrinsics,
    Vector2f(camera_params[1].depth.resolution),
    camera_params[1].depth.depth_range.left(),
    camera_params[1].depth.depth_range.right()
  );
  Vector4f p0 = center_camera.worldFromScreen(xy0,
    z0, Vector2f(camera_params[1].depth.resolution));

  constexpr int kRegularGridResolution = 512;
  constexpr float kRegularGridSideLength = 2.0f;
  constexpr float kVoxelSize = kRegularGridSideLength / kRegularGridResolution;
  constexpr float kMaxTSDFValue = 32 * kVoxelSize;
  const SimilarityTransform initial_world_from_grid =
    MakeWorldFromGrid(p0.xyz, kRegularGridSideLength, kRegularGridResolution);

  MultiStaticCameraPipeline pipeline(camera_params, camera_poses,
                                     Vector3i{kRegularGridResolution},
                                     initial_world_from_grid, kMaxTSDFValue);
  if (FLAGS_ms_use_gui) {
    ControlWidget control_widget;
    control_widget.setGeometry(50, 50, 150, 150);
    control_widget.show();

    MainWidget main_widget;
    main_widget.SetPipeline(&pipeline);

    const int window_width = 1920;
    const int window_height = 1200;
    int x = control_widget.frameGeometry().right();
    int y = control_widget.frameGeometry().top();
    main_widget.move(x, y);
    main_widget.resize(window_width, window_height);
    main_widget.show();

    // HACK:
    // GUI version, non-gui version
    MainController controller(nullptr, nullptr,
                              &control_widget, &main_widget);
    for(size_t i = 0; i < rgbd_stream_filenames.size(); ++i) {
      controller.inputs_.emplace_back(RgbdInput::InputType::FILE,
                                      rgbd_stream_filenames[i].c_str());
    }
    controller.msc_pipeline_ = &pipeline;
    return app.exec();
  } else {
    std::vector<RgbdInput> inputs;
    for(size_t i = 0; i < rgbd_stream_filenames.size(); ++i) {
      inputs.emplace_back(RgbdInput::InputType::FILE,
        rgbd_stream_filenames[i].c_str());
    }

    NumberedFilenameBuilder nfb("c:/tmp/multicam/meshes/frame_", ".obj");

    int frame_index = 0;
    while (true) {
      bool rgb_updated;
      bool depth_updated;
      for(int i = 0; i < static_cast<int>(inputs.size()); ++i) {
        printf("Processing frame %d of %zu\n", i, inputs.size());
        pipeline.Reset();

        inputs[i].read(&pipeline.GetInputBuffer(i),
          &rgb_updated, &depth_updated);
        if (!depth_updated) {
          break;
        }

        pipeline.NotifyInputUpdated(i, rgb_updated, depth_updated);
      }
      // TODO: shouldn't have to call this.
      pipeline.Fuse();
      pipeline.Triangulate(rot180.asMatrix()).saveOBJ(
      nfb.filenameForNumber(frame_index));
      ++frame_index;
      if (frame_index > 1799) {
        break;
      }
    }

    return 0;
  }
}


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_mode == "single_moving") {
    return SingleMovingCameraMain(argc, argv);
  } else if (FLAGS_mode == "multi_static") {
    return MultiStaticCameraMain(argc, argv);
  } else {
    printf("Invalid mode: %s.\n"
      "mode must be \"single_moving\" or \"multi_static\"\n",
      FLAGS_mode.c_str());
    return 1;
  }
}

