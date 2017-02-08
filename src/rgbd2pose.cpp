#include <gflags/gflags.h>

#include <string>

#include "libcgt/core/vecmath/Vector2i.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"
#include "libcgt/camera_wrappers/PoseStream.h"

#include "rgbd_camera_parameters.h"
#include "rgbd_input.h"
#include "input_buffer.h"

#include "aruco/cube_fiducial.h"
#include "aruco_pose_estimator.h"

using libcgt::camera_wrappers::PoseInputStream;
using libcgt::camera_wrappers::PoseOutputStream;
using libcgt::camera_wrappers::PoseStreamFormat;
using libcgt::camera_wrappers::PoseStreamMetadata;
using libcgt::camera_wrappers::PoseStreamTransformDirection;
using libcgt::camera_wrappers::PoseStreamUnits;
using libcgt::core::vecmath::EuclideanTransform;

DEFINE_string(input_file, "", "Input .rgbd file.");

DEFINE_string(calibration_dir, "",
  "calibration directory for the RGBD camera.");

DEFINE_string(output_file, "", "output .pose file.");

DEFINE_bool(collect_perf, false, "Collect performance statistics.");

const char* kArucoDetectorParamsFilename = "../res/detector_params.yaml";

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_input_file == "") {
    printf("input_file is required.\n");
    return 1;
  }
  if (FLAGS_calibration_dir == "") {
    printf("calibration_dir is required.\n");
    return 1;
  }
  if (FLAGS_output_file == "") {
    printf("output_file is required.\n");
    return 1;
  }

  // TODO: tweak this so that it can fail.
  RGBDCameraParameters camera_params =
    LoadRGBDCameraParameters(FLAGS_calibration_dir);

  bool rgb_updated;
  bool depth_updated;
  RgbdInput rgbd_input(RgbdInput::InputType::FILE, FLAGS_input_file.c_str());

  InputBuffer input_buffer(
    camera_params.color.resolution,
    camera_params.depth.resolution);

  CubeFiducial fiducial;
  ArucoPoseEstimator pose_estimator(
    fiducial,
    camera_params.color,
    kArucoDetectorParamsFilename);

  PoseStreamMetadata metadata;
  metadata.format = PoseStreamFormat::ROTATION_MATRIX_3X3_COL_MAJOR_AND_TRANSLATION_VECTOR_FLOAT;
  metadata.units = PoseStreamUnits::METERS;
  metadata.direction = PoseStreamTransformDirection::CAMERA_FROM_WORLD;

  PoseOutputStream output_stream(metadata, FLAGS_output_file.c_str());

  // Read initial frame.
  rgbd_input.read(&input_buffer, &rgb_updated, &depth_updated);
  while (rgb_updated || depth_updated) {
    if (rgb_updated) {
      printf("Processing frame. idx = %d, timestamp = %lld\n",
        input_buffer.color_frame_index,
        input_buffer.color_timestamp_ns);
      auto result = pose_estimator.EstimatePose(input_buffer.color_bgr_ydown);
      if (result.valid) {
        auto cfw = inverse(result.world_from_camera);
        printf("translation = %s\n", result.world_from_camera.translation.toString().c_str());
        output_stream.write(
          input_buffer.color_frame_index,
          input_buffer.color_timestamp_ns,
          cfw.rotation,
          cfw.translation
        );
      } else {
        printf("Failed to find pose.\n");
      }
    }

    // Read next frame.
    rgbd_input.read(&input_buffer, &rgb_updated, &depth_updated);
  }
}
