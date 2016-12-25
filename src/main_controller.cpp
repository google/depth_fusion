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
#include <QTimer>

#include "libcgt/camera_wrappers/PoseStream.h"

#include "control_widget.h"
#include "main_widget.h"
#include "rgbd_input.h"

DECLARE_string(mode);

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
  QObject::connect(control_widget, &ControlWidget::resetClicked,
    this, &MainController::OnResetClicked);
  QObject::connect(control_widget, &ControlWidget::saveMeshClicked,
    this, &MainController::OnSaveMeshClicked);
  QObject::connect(control_widget, &ControlWidget::savePoseClicked,
    this, &MainController::OnSavePoseClicked);

  if (pipeline != nullptr) {
    QObject::connect(pipeline, &RegularGridFusionPipeline::dataChanged,
      main_widget_->GetSingleMovingCameraGLState(),
      &SingleMovingCameraGLState::OnPipelineDataChanged);
  }
}

void MainController::OnReadInput() {
  if (FLAGS_mode == "single_moving") {
    bool color_updated;
    bool depth_updated;
    input_->read(&(pipeline_->GetInputBuffer()), &color_updated, &depth_updated);
    pipeline_->NotifyInputUpdated(color_updated, depth_updated);
  } else if (FLAGS_mode == "multi_static") {
    for (size_t i = 0; i < inputs_.size(); ++i) {
      bool rgb_updated;
      bool depth_updated;
      inputs_[i].read(&(smc_pipeline_->GetInputBuffer(i)),
        &rgb_updated, &depth_updated);
      smc_pipeline_->NotifyInputUpdated(i, rgb_updated, depth_updated);
    }

    //if(depth_updated) {
    {
      // TODO: pipeline should emit that TSDF has changed.
      smc_pipeline_->Fuse();
      main_widget_->GetStaticMultiCameraGLState()->NotifyTSDFUpdated();
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
  if (pipeline_ != nullptr) {
    pipeline_->Reset();
  }
  if (smc_pipeline_ != nullptr) {
    smc_pipeline_->Reset();
  }
}

void MainController::OnSaveMeshClicked(QString filename) {
  if (FLAGS_mode == "single_moving") {
    TriangleMesh mesh = pipeline_->Triangulate();
    bool save_succeeded = mesh.saveOBJ(filename.toStdString().c_str());
  } else if (FLAGS_mode == "multi_static") {
    // HACK: rot180
    Matrix4f rot180 = Matrix4f::rotateX(static_cast<float>(M_PI));
    TriangleMesh mesh = smc_pipeline_->Triangulate(rot180);
    bool save_succeeded = mesh.saveOBJ(filename.toStdString().c_str());
  }
  // TODO: messagebox for success
}

void SavePoses(QString filename, const std::vector<PoseFrame> poses) {
  // TODO: implement me.
#if 0
  libcgt::camera_wrappers::PoseStreamMetadata metadata;
  metadata.direction = libcgt::camera_wrappers::PoseStreamTransformDirection::CAMERA_FROM_WORLD;
  metadata.format = libcgt::camera_wrappers::PoseStreamFormat::ROTATION_MATRIX_3X3_COL_MAJOR_AND_TRANSLATION_VECTOR_FLOAT;

  libcgt::camera_wrappers::PoseOutputStream output_stream(metadata,
    filename.toStdString().c_str());

  // TODO(jiawen): messagebox for success
  for (size_t i = 0; i < poses.size(); ++i) {
    bool succeeded = output_stream.write(
      poses[i].frame_index, poses[i].timestamp_ns,
      poses[i].camera_from_world.rotation,
      poses[i].camera_from_world.translation);
  }
#endif
}

void MainController::OnSavePoseClicked(QString filename) {
  if (filename != "") {
    SavePoses(filename, pipeline_->PoseHistory());
  }
}
