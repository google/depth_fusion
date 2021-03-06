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
#include "control_widget.h"

#include <QFileDialog>
#include <QGridLayout>
#include <QPushButton>

ControlWidget::ControlWidget(QWidget* parent) :
  QWidget(parent) {
  QGridLayout* layout = new QGridLayout;

  QPushButton* pause_button = new QPushButton("Pause / Resume");
  QObject::connect(pause_button, &QPushButton::clicked,
    this, &ControlWidget::pauseClicked);
  layout->addWidget(pause_button);

  QPushButton* step_frame_button = new QPushButton("Step One Frame");
  QObject::connect(step_frame_button, &QPushButton::clicked,
    this, &ControlWidget::stepClicked);
  layout->addWidget(step_frame_button);

  QPushButton* reset_button = new QPushButton("Reset");
  QObject::connect( reset_button, &QPushButton::clicked,
    this, &ControlWidget::resetClicked);
  layout->addWidget(reset_button);

  QPushButton* save_mesh_button = new QPushButton("Save Mesh");
  QObject::connect(save_mesh_button, &QPushButton::clicked,
    this, &ControlWidget::OnSaveMeshClicked);
  layout->addWidget(save_mesh_button);

  QPushButton* save_pose_button = new QPushButton("Save Pose Stream");
  QObject::connect(save_pose_button, &QPushButton::clicked,
    this, &ControlWidget::OnSavePoseClicked);
  layout->addWidget(save_pose_button);

  setLayout(layout);
}

void ControlWidget::OnSaveMeshClicked() {
  QString filename = QFileDialog::getSaveFileName(this,
    "Save Mesh",
    QString(),
    "Alias|Wavefront Meshes (*.obj)"
    );
  if (filename != "") {
    emit saveMeshClicked(filename);
  }
}

void ControlWidget::OnSavePoseClicked() {
  QString filename = QFileDialog::getSaveFileName(this,
    "Save Pose Stream",
    QString(),
    "Pose Streams (*.pose)"
  );

  if (filename != "") {
    emit savePoseClicked(filename);
  }
}
