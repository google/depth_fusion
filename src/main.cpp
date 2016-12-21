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
#include <string>

#include <QApplication>

#include <camera_wrappers/StreamConfig.h>
#include <core/vecmath/EuclideanTransform.h>
#include <core/vecmath/SimilarityTransform.h>
#include <core/vecmath/Quat4f.h>
#include <io/NumberedFilenameBuilder.h>
#include <opencv_interop/Calib3d.h>
#include <opencv_interop/VecmathUtils.h>
#include <gflags/gflags.h>

#include "control_widget.h"
#include "input_buffer.h"
#include "main_widget.h"
#include "main_controller.h"
#include "regular_grid_fusion_pipeline.h"
#include "rgbd_camera_parameters.h"
#include "rgbd_input.h"

using libcgt::opencv_interop::cameraMatrixCVToGL;
using libcgt::opencv_interop::cameraMatrixToIntrinsics;
using libcgt::opencv_interop::makeCameraMatrix;
using libcgt::opencv_interop::undistortRectifyMap;
using libcgt::opencv_interop::toCVSize;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::SimilarityTransform;
using std::string;

DEFINE_bool(multi_cam_gui, false,
            "If true, run multi-camera GUI app.");
DEFINE_bool(run_multi_cam, true,
            "If true, use the multi-camera static pipeline.");

DEFINE_double(max_tsdf_value_scale, 32.0,
              "Maximum TSDF value used in the voxel grid. "
              "The value is measured in multiples of voxel side lengths.");

DEFINE_string(output_mesh_sequence_prefix, 
  "/usr/local/google/home/rmbrualla/projects/depth_fusion/data/dan6/frame_",
  "Output location of meshes when using non-gui mode.");

DEFINE_string(output_mesh_sequence_suffix, 
  "",
  "Output location of meshes when using non-gui mode.");


DEFINE_int32(process_only_frame, -1,
             "If different than -1, process only the i-th frame.");
DEFINE_int32(grid_resolution, 512,
             "Resolution of voxel grid used for fusion.");
DEFINE_double(grid_side_length, 2.0f,
              "Side length of the regular grid used for fusion.");

int SingleMovingCameraMain(int argc, char* argv[]) {
  // TODO: use gflags
  if (argc < 3) {
    printf("Usage: %s <calibration_dir> <rgbd_file>\n", argv[0]);
    return 1;
  }

  QApplication app(argc, argv);

  // TODO: switch these to use gflags.
  RGBDCameraParameters camera_params =
      LoadRGBDCameraParameters(argv[1]);

  std::string rgbd_stream_filename = argv[2];

  const Vector3i kRegularGridResolution(512); // ~2m^3
  const float kRegularGridVoxelSize = 0.004f; // 4 mm.

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

#if 0
  RgbdInput rgbd_input(RgbdInput::InputType::FILE,
                       rgbd_stream_filename.c_str());
#else
  RgbdInput rgbd_input(RgbdInput::InputType::OPENNI2, nullptr);
#endif

  PoseFrame initial_pose_frame = {};
  initial_pose_frame.method = PoseFrame::EstimationMethod::FIXED_INITIAL;
  initial_pose_frame.depth_camera_from_world = kInitialDepthCameraFromWorld;
  initial_pose_frame.color_camera_from_world =
    camera_params.ConvertToColorCameraFromWorld(kInitialDepthCameraFromWorld);

  RegularGridFusionPipeline pipeline(camera_params,
    kRegularGridResolution, kInitialWorldFromGrid, initial_pose_frame);

  // TODO: pipeline->SetTracker()...
  // TODO: add different color pose tracking options using gflags.
  ControlWidget control_widget;
  control_widget.setGeometry(50, 50, 150, 150);
  control_widget.show();

  MainWidget main_widget;
  main_widget.SetPipeline(&pipeline);
  main_widget.show();

  const int window_width = 2240;
  const int window_height = 1200;
  int x = control_widget.geometry().right();
  int y = control_widget.geometry().top();
  main_widget.move(x, y);
  main_widget.resize(window_width, window_height);

  MainController controller(&rgbd_input, &pipeline,
    &control_widget, &main_widget);
  return app.exec();
}


// HACK: this only makes a depth camera.
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

int StaticMultiCameraMain(int argc, char* argv[]) {
  QApplication app(argc, argv);

  std::vector<std::string> rgbd_stream_filenames = {
    "/usr/local/google/home/rmbrualla/projects/depth_fusion/data/dan6/kinect0083_new/filtered_frames.rgbd",
    "/usr/local/google/home/rmbrualla/projects/depth_fusion/data/dan6/kinect0134_new/filtered_frames.rgbd",
    "/usr/local/google/home/rmbrualla/projects/depth_fusion/data/dan6/kinect0142_new/filtered_frames.rgbd"};

  const int kNumCameras = 3;
  std::vector<RGBDCameraParameters> camera_params(3);
  camera_params[0] = makeRGBDCameraParameters(365.183, 255.097, 207.695,
    0.0745901, -0.189055);
  camera_params[1] = makeRGBDCameraParameters(366.009, 251.421, 205.132,
    0.0641811, -0.179726);
  camera_params[2] = makeRGBDCameraParameters(365.712, 254.596, 205.857,
    0.0678888, -0.185925);

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
    camera_params[1].depth.resolution,
    camera_params[1].depth.depth_range.left(),
    camera_params[1].depth.depth_range.right()
  );
  Vector4f p0 = center_camera.worldFromScreen(xy0,
    z0, camera_params[1].depth.resolution);

  const int kRegularGridResolution = FLAGS_grid_resolution;
  const float kRegularGridSideLength = FLAGS_grid_side_length;
  const float kVoxelSize = kRegularGridSideLength / kRegularGridResolution;
  const float kMaxTSDFValue = FLAGS_max_tsdf_value_scale * kVoxelSize;
  const SimilarityTransform initial_world_from_grid =
    MakeWorldFromGrid(p0.xyz, kRegularGridSideLength, kRegularGridResolution);

  StaticMultiCameraPipeline pipeline(camera_params, camera_poses,
                                     Vector3i{kRegularGridResolution},
                                     initial_world_from_grid, kMaxTSDFValue);
  if (FLAGS_multi_cam_gui) {
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
    controller.smc_pipeline_ = &pipeline;
    return app.exec();
  } else {
    // Run as command line tool.
    std::vector<RgbdInput> inputs;
    for(size_t i = 0; i < rgbd_stream_filenames.size(); ++i) {
      inputs.emplace_back(RgbdInput::InputType::FILE,
        rgbd_stream_filenames[i].c_str());
    }

    NumberedFilenameBuilder nfb(FLAGS_output_mesh_sequence_prefix,
                                FLAGS_output_mesh_sequence_suffix + ".obj");

    int frame_index = 0;
    while (true) {
      bool rgb_updated;
      bool depth_updated;
      pipeline.Reset();

      for(size_t i = 0; i < inputs.size(); ++i) {
        printf("Processing frame %zu of %zu\n", i, inputs.size());

        inputs[i].read(&pipeline.GetInputBuffer(i),
          &rgb_updated, &depth_updated);
        if (!depth_updated) {
          break;
        }

        pipeline.NotifyInputUpdated(i, rgb_updated, depth_updated);
      }
      // TODO: shouldn't have to call this.
      if (FLAGS_process_only_frame == frame_index || FLAGS_process_only_frame == -1) {
        pipeline.Fuse();
        std::string filename = nfb.filenameForNumber(frame_index);
        pipeline.Triangulate(rot180.asMatrix()).saveOBJ(filename);
        if (FLAGS_process_only_frame == frame_index) return 0;
      }
      ++frame_index;
      if (frame_index > 1799) {
        break;
      }
    }

  return 0;
  }
}

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  glewInit();

  if (FLAGS_run_multi_cam) {
    return StaticMultiCameraMain(argc, argv);
  } else {
    return SingleMovingCameraMain(argc, argv);
  }
}
