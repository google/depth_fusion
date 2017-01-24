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

DEFINE_string(mode, "single_moving",
  "Mode to run the app in. Either \"single_moving\" or \"multi_static\"." );
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
  "\"color_aruco_and_depth_icp\", or \"file\". "
  "If \"file\", sm_pose_file is required.");
DEFINE_string(sm_pose_file, "",
  "Filename for precomputed pose path.");


DEFINE_bool(ms_use_gui, true,
  "Set true to visualize with GUI, false to run in batch mode.");
DEFINE_string(ms_camera_calibration_file, "",
              "Filename of text file containing description of camera pose "
              "and intrinsic parameters. See code for format details.");
DEFINE_string(ms_rgbd_base_dir, "",
              "Base directory where rgbd files are located.");
DEFINE_string(ms_rgbd_sequence_suffix, "",
              "Suffix after first \"_\" in camera name to be added "
              "when generating the rgdb filenames to be loaded.");

DEFINE_double(ms_max_tsdf_value_scale, 32.0,
              "Maximum TSDF value used in the voxel grid. "
              "The value is measured in multiples of voxel side lengths.");
DEFINE_int32(ms_grid_resolution, 512,
             "Resolution of voxel grid used for fusion.");
DEFINE_double(ms_grid_side_length, 2.0f,
              "Side length of the regular grid used for fusion.");
DEFINE_double(ms_grid_distance_z, 2.5, "Distance in z coordinate of the grid.");
DEFINE_double(ms_grid_offset_x, 0.0, "Offset of the grid in x coordinate from the optical axis.");
DEFINE_double(ms_grid_offset_y, 0.0, "Offset of the grid in y coordinate from the optical axis..");
DEFINE_double(ms_depth_range_min, 0.1, "Minimum depth considered in meters.");
DEFINE_double(ms_depth_range_max, 10.0, "Maximum depth considered in meters.");


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

  RGBDCameraParameters camera_params =
    LoadRGBDCameraParameters(FLAGS_sm_calibration_dir);

  const Vector3i kRegularGridResolution(512); // ~2m^3
  const float kRegularGridVoxelSize = 0.004f; // 4 mm.

  RgbdInput rgbd_input(input_type, FLAGS_sm_input_args.c_str());

  std::unique_ptr<RegularGridFusionPipeline> pipeline;

  if (FLAGS_sm_pose_estimator == "color_aruco" ||
    FLAGS_sm_pose_estimator == "color_aruco_and_depth_icp") {
    // Put the origin at the center of the cube.
    const SimilarityTransform kInitialWorldFromGrid =
      SimilarityTransform(kRegularGridVoxelSize) *
      SimilarityTransform(Vector3f(-0.5f * kRegularGridResolution.x,
        -0.5f * kRegularGridResolution.y, -0.5f * kRegularGridResolution.z));

    PoseFrame::EstimationMethod estimator;
    if (FLAGS_sm_pose_estimator == "color_aruco") {
      estimator = PoseFrame::EstimationMethod::COLOR_ARUCO;
    } else {
      estimator = PoseFrame::EstimationMethod::COLOR_ARUCO_AND_DEPTH_ICP;
    }

    pipeline = std::make_unique<RegularGridFusionPipeline>(camera_params,
      kRegularGridResolution, kInitialWorldFromGrid, estimator);
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

    PoseFrame initial_pose_frame = {};
    initial_pose_frame.method = PoseFrame::EstimationMethod::FIXED_INITIAL;
    initial_pose_frame.depth_camera_from_world = kInitialDepthCameraFromWorld;
    initial_pose_frame.color_camera_from_world =
      camera_params.ConvertToColorCameraFromWorld(
        kInitialDepthCameraFromWorld);

    pipeline = std::make_unique<RegularGridFusionPipeline>(camera_params,
      kRegularGridResolution, kInitialWorldFromGrid,
      PoseFrame::EstimationMethod::DEPTH_ICP, initial_pose_frame);
  } else if (FLAGS_sm_pose_estimator == "precomputed") {
    // Put the origin at the center of the cube.
    const SimilarityTransform kInitialWorldFromGrid =
      SimilarityTransform(kRegularGridVoxelSize) *
      SimilarityTransform(Vector3f(-0.5f * kRegularGridResolution.x,
        -0.5f * kRegularGridResolution.y, -0.5f * kRegularGridResolution.z));

    std::vector<PoseFrame> pose_history = LoadPoseHistory(FLAGS_sm_pose_file,
      camera_params.depth_from_color);
    if (pose_history.size() == 0) {
      fprintf(stderr, "Error: failed to load pose history from %s\n",
        FLAGS_sm_pose_file.c_str());
      return 1;
    }

    pipeline = std::make_unique<RegularGridFusionPipeline>(camera_params,
      kRegularGridResolution, kInitialWorldFromGrid, pose_history);
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

// HACK: this only makes a depth camera.
RGBDCameraParameters makeRGBDCameraParameters(
  double focalLengthX, double focalLengthY,
  double principalPointX, double principalPointY,
  double distortionR2, double distortionR4,
  int image_width, int image_height) {
  RGBDCameraParameters params;

  params.depth.resolution = { image_width, image_height };
  params.depth.depth_range = Range1f::fromMinMax(FLAGS_ms_depth_range_min, FLAGS_ms_depth_range_max);

  cv::Mat_<double> cvCameraMatrix = makeCameraMatrix(focalLengthX,
    principalPointX, principalPointY);
  cvCameraMatrix(1, 1) = focalLengthY;

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

  int num_cameras = 0;

  ifstream model_cameras_file(FLAGS_ms_camera_calibration_file.c_str());

  model_cameras_file >> num_cameras;
  printf("Reading %d cameras...\n", num_cameras);
  std::vector<std::string> rgbd_stream_filenames(num_cameras);
  std::vector<RGBDCameraParameters> camera_params(num_cameras);
  std::vector<EuclideanTransform> camera_poses(num_cameras);

  for (int i = 0; i < num_cameras; i++) {
    string id;
    model_cameras_file >> id;
    printf("Processing camera %s\n", id.c_str());

    string base_dir = FLAGS_ms_rgbd_base_dir;
    // HACK: won't work in Windows.
    if (base_dir[base_dir.size() -1] != '/') base_dir += '/';
    rgbd_stream_filenames[i] = base_dir + id.substr(0, id.find('_')) +
                               FLAGS_ms_rgbd_sequence_suffix +
                               id.substr(id.find('_')) + ".rgbd";
    printf("file: %s\n", rgbd_stream_filenames[i].c_str());

    double focal_length_x, focal_length_y;
    double principal_point_x, principal_point_y;
    double radial_distortion_r2, radial_distortion_r4;
    int image_width, image_height;

    model_cameras_file >> focal_length_x >> focal_length_y 
                       >> principal_point_x >> principal_point_y
                       >> radial_distortion_r2 >> radial_distortion_r4
                       >> image_width >> image_height;
    camera_params[i] = 
        makeRGBDCameraParameters(focal_length_x, focal_length_y,
                                 principal_point_x, principal_point_y,
                                 radial_distortion_r2, radial_distortion_r4,
                                 image_width, image_height);


    Vector3f translation, axis_angle;
    model_cameras_file >> translation.x >> translation.y >> translation.z;
    model_cameras_file >> axis_angle.x >> axis_angle.y >> axis_angle.z;
    camera_poses[i].translation = translation;
    camera_poses[i].rotation = Matrix3f::rotation(axis_angle);
  }


  for(int i = 0; i < num_cameras; ++i) {
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
    camera_params[1].depth.resolution.x * (0.5f + static_cast<float>(FLAGS_ms_grid_offset_x)),
    camera_params[1].depth.resolution.y * (0.5f + static_cast<float>(FLAGS_ms_grid_offset_y))};

  const float z0 = FLAGS_ms_grid_distance_z;
  PerspectiveCamera center_camera(
    camera_poses[1],
    camera_params[1].depth.intrinsics,
    camera_params[1].depth.resolution,
    camera_params[1].depth.depth_range.left(),
    camera_params[1].depth.depth_range.right()
  );
  Vector4f p0 = center_camera.worldFromScreen(xy0,
    z0, camera_params[1].depth.resolution);

  const int kRegularGridResolution = FLAGS_ms_grid_resolution;
  const float kRegularGridSideLength = FLAGS_ms_grid_side_length;
  const float kVoxelSize = kRegularGridSideLength / kRegularGridResolution;
  const float kMaxTSDFValue = FLAGS_ms_max_tsdf_value_scale * kVoxelSize;
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
      for(size_t i = 0; i < inputs.size(); ++i) {
        printf("Processing frame %zu of %zu\n", i, inputs.size());
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
    printf("Invalid mode: %s\n", FLAGS_mode.c_str());
    return 1;
  }
}
