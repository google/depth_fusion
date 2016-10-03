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

#include <QTimer>

#include <camera_wrappers/PoseStream.h>

#include "control_widget.h"
#include "main_widget.h"
#include "rgbd_input.h"

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
  QObject::connect(control_widget, &ControlWidget::saveMeshClicked,
    this, &MainController::OnSaveMeshClicked);
  QObject::connect(control_widget, &ControlWidget::savePoseClicked,
    this, &MainController::OnSavePoseClicked);
}

void MainController::OnReadInput() {
  bool rgb_updated;
  bool depth_updated;
  input_->read(&(pipeline_->GetInputBuffer()), &rgb_updated, &depth_updated);

  pipeline_->NotifyInputUpdated(rgb_updated, depth_updated);

  bool tracking_succeeded = false;
  if (depth_updated) {
    // put frameId and timestamps into history and input

    tracking_succeeded = pipeline_->UpdatePoseWithICP();
    if (tracking_succeeded) {
      pipeline_->Fuse();
      pipeline_->Raycast();
    }
  }

  if (rgb_updated || depth_updated) {
    main_widget_->update();
  }

  if (tracking_succeeded) {
    // main_widget->...notify raycast changed
  }
}

void MainController::OnPauseClicked() {
  if (read_input_timer_->isActive()) {
    read_input_timer_->stop();
  } else {
    read_input_timer_->start();
  }
}

void MainController::OnSaveMeshClicked(QString filename) {
  TriangleMesh mesh = pipeline_->Triangulate();
  bool save_succeeded = mesh.saveOBJ(filename.toStdString().c_str());
  // TODO(jiawen): messagebox for success
}

void SavePoses(QString filename, const std::vector<PoseFrame> poses) {
  libcgt::camera_wrappers::PoseStreamMetadata metadata;
  metadata.direction = libcgt::camera_wrappers::PoseStreamTransformDirection::CAMERA_FROM_WORLD;
  metadata.format = libcgt::camera_wrappers::PoseStreamFormat::ROTATION_MATRIX_3X3_COL_MAJOR_AND_TRANSLATION_VECTOR_FLOAT;

  libcgt::camera_wrappers::PoseOutputStream output_stream(metadata,
    filename.toStdString().c_str());

  // TODO(jiawen): messagebox for success
  for (size_t i = 0; i < poses.size(); ++i) {
    bool succeeded = output_stream.write(
      poses[i].frame_index, poses[i].timestamp,
      poses[i].camera_from_world.rotation,
      poses[i].camera_from_world.translation);
  }
}

void MainController::OnSavePoseClicked(QString depth_filename,
  QString color_filename) {
  if (depth_filename != "") {
    const auto& depth_poses = pipeline_->DepthPoseHistory();
    SavePoses(depth_filename, depth_poses);
  }
  if (color_filename != "") {
    const auto& color_poses = pipeline_->ColorPoseHistoryEstimatedDepth();
    SavePoses(color_filename, color_poses);
  }
}
