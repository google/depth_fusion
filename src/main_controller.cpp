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
#include "main_controller.h"

#include <gflags/gflags.h>
#include <QMessageBox>
#include <QTimer>

#include "libcgt/core/common/ArrayUtils.h"
#include "libcgt/core/io/PortableFloatMapIO.h"
#include "libcgt/camera_wrappers/PoseStream.h"

#include "control_widget.h"
#include "main_widget.h"
#include "pose_utils.h"
#include "rgbd_input.h"

DECLARE_string(mode);
DECLARE_string(sm_input_type);

// Outputs.
DECLARE_string(sm_output_mesh);
DECLARE_string(sm_output_pose);
#if 0
DECLARE_string(sm_output_rgbd);
#endif
DECLARE_string(sm_output_dir);

using libcgt::core::arrayutils::cast;
using libcgt::core::arrayutils::flipY;

namespace {
  constexpr int kTimestampFieldWidth = 20;
}

MainController::MainController(
  RgbdInput* input, RegularGridFusionPipeline* pipeline,
  ControlWidget* control_widget, MainWidget* main_widget) :
  input_(input),
  pipeline_(pipeline),
  main_widget_(main_widget) {
  read_input_timer_ = new QTimer(this);
  read_input_timer_->setInterval(0);
  QObject::connect(read_input_timer_, &QTimer::timeout,
    this, &MainController::OnReadInput);

  QObject::connect(control_widget, &ControlWidget::pauseClicked,
    this, &MainController::OnPauseClicked);
  QObject::connect(control_widget, &ControlWidget::stepClicked,
    this, &MainController::OnReadInput);
  QObject::connect(control_widget, &ControlWidget::resetClicked,
    this, &MainController::OnResetClicked);
  QObject::connect(control_widget, &ControlWidget::saveMeshClicked,
    this, &MainController::OnSaveMeshClicked);
  QObject::connect(control_widget, &ControlWidget::savePoseClicked,
    this, &MainController::OnSavePoseClicked);
  QObject::connect(pipeline, &RegularGridFusionPipeline::dataChanged,
    main_widget_->GetSingleMovingCameraGLState(),
    &SingleMovingCameraGLState::OnPipelineDataChanged);
}

void MainController::OnReadInput() {
  static int i = 0;
  if (FLAGS_mode == "single_moving") {
    bool color_updated;
    bool depth_updated;
    // TODO: read should report a status:
    // COLOR_UPDATED, DEPTH_UPDATED, TIMEOUT or END_OF_FILE.
    input_->read(&(pipeline_->GetInputBuffer()),
      &color_updated, &depth_updated);
    // TODO: read just color or depth, not both.
    if (color_updated || depth_updated) {
      if (color_updated) {
        pipeline_->NotifyColorUpdated();
      } else if (depth_updated) {
        pipeline_->NotifyDepthUpdated();
      }
    } else if (FLAGS_sm_input_type == "file") {
      read_input_timer_->stop();
      OnEndOfStream();
    }
  } else if (FLAGS_mode == "multi_static") {
    for (int i = 0; i < static_cast<int>(inputs_.size()); ++i) {
      bool color_updated;
      bool depth_updated;
      inputs_[i].read(&(msc_pipeline_->GetInputBuffer(i)),
        &color_updated, &depth_updated);
      if (color_updated || depth_updated) {
        msc_pipeline_->NotifyInputUpdated(i, color_updated, depth_updated);
      } else if (FLAGS_sm_input_type == "file") {
        read_input_timer_->stop();
      }
    }

    //if(depth_updated) {
    {
      // TODO: pipeline should emit that TSDF has changed.
      msc_pipeline_->Reset();
      msc_pipeline_->FuseMultiple();
      main_widget_->GetMultiStaticCameraGLState()->NotifyTSDFUpdated();
    }

    //if(rgb_updated || depth_updated) {
    {
      main_widget_->update();
    }
  }
}

void MainController::OnPauseClicked() {
  if (read_input_timer_->isActive()) {
    read_input_timer_->stop();
  } else {
    read_input_timer_->start();
  }
}

void MainController::OnResetClicked() {
  // TODO: reset rgbd and pose streams.
  if (pipeline_ != nullptr) {
    pipeline_->Reset();
  }
  if (msc_pipeline_ != nullptr) {
    msc_pipeline_->Reset();
  }
}

void MainController::OnSaveMeshClicked(QString filename) {
  if (FLAGS_mode == "single_moving") {
    TriangleMesh mesh = pipeline_->Triangulate();
    bool succeeded = mesh.saveOBJ(filename.toStdString().c_str());
    if (!succeeded) {
      QMessageBox::critical(main_widget_, "Save Mesh Status",
        "Failed to save to: " + filename);
    }
  } else if (FLAGS_mode == "multi_static") {
    // HACK: rot180
    Matrix4f rot180 = Matrix4f::rotateX(static_cast<float>(M_PI));
    TriangleMesh mesh = msc_pipeline_->Triangulate(rot180);
    bool succeeded = mesh.saveOBJ(filename.toStdString().c_str());
    if (!succeeded) {
      QMessageBox::critical(main_widget_, "Save Mesh Status",
        "Failed to save to: " + filename);
    }
  }
}

void MainController::OnSavePoseClicked(QString filename) {
  if (filename != "") {
    printf("Saving pose stream to %s...", filename.toStdString().c_str());
    bool succeeded = SavePoseHistory(
      pipeline_->PoseHistory(), filename.toStdString());
    if (succeeded) {
      printf("succeeded.\n");
    } else {
      printf("failed.\n");
      QMessageBox::critical(main_widget_, "Save Pose Stream Status",
        "Failed to save to: " + filename);
    }
  }
}

void MainController::OnEndOfStream() {
  // Read pose frames from file.
  std::string pose_filename = "d:/tmp/slf/still_life_00002/sfm_aligned_to_aruco.pose";
  const auto& rgbdParams = pipeline_->GetCameraParameters();
  const auto& colorParams = rgbdParams.color;
  std::vector<PoseFrame> raycast_poses = LoadPoseHistory(pose_filename,
    rgbdParams.depth_from_color);

  // Read intrinsics from file?

  // TODO: save volume.

  if (FLAGS_sm_output_dir != "") {
    // TODO: separate timestamps for precomputed poses.

    PerspectiveCamera camera;
    camera.setFrustum(colorParams.undistorted_intrinsics,
      Vector2f(colorParams.resolution));

    Array2D<Vector4f> world_points(colorParams.resolution);
    Array2D<Vector4f> world_normals(colorParams.resolution);
    DeviceArray2D<float4> device_world_points(colorParams.resolution);
    DeviceArray2D<float4> device_world_normals(colorParams.resolution);

    for (const PoseFrame& p : raycast_poses) {
      char world_points_filename[255];
      char world_normals_filename[255];
      sprintf(world_points_filename,"%s/world_points_%05d_%020lld.pfm4",
        FLAGS_sm_output_dir.c_str(),
        p.frame_index, p.timestamp_ns);
      sprintf(world_normals_filename, "%s/world_normals_%05d_%020lld.pfm4",
        FLAGS_sm_output_dir.c_str(),
        p.frame_index, p.timestamp_ns);

      camera.setCameraFromWorld(p.color_camera_from_world);
      pipeline_->Raycast(camera, device_world_points, device_world_normals);

      copy(device_world_points, cast<float4>(world_points.writeView()));
      copy(device_world_normals, cast<float4>(world_normals.writeView()));

      PortableFloatMapIO::write(flipY(world_points.readView()),
        world_points_filename);
      PortableFloatMapIO::write(flipY(world_normals.readView()),
        world_normals_filename);
    }
  }

  if (FLAGS_sm_output_mesh != "") {
    OnSaveMeshClicked(QString::fromStdString(FLAGS_sm_output_mesh));
  }
  if (FLAGS_sm_output_pose != "") {
    OnSavePoseClicked(QString::fromStdString(FLAGS_sm_output_pose));
  }
}
