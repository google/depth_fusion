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
#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include <QObject>
#include <QString>

#include "regular_grid_fusion_pipeline.h"
#include "static_multi_camera_pipeline.h"

class ControlWidget;
class MainWidget;
class QTimer;
class RgbdInput;

class MainController : public QObject {

  Q_OBJECT

 public:

   MainController(RgbdInput* input, RegularGridFusionPipeline* pipeline,
     ControlWidget* control_widget, MainWidget* main_widget);

   // HACK
   std::vector<RgbdInput> inputs_;
   StaticMultiCameraPipeline* smc_pipeline_ = nullptr;

 public slots:

  void OnPauseClicked();
  void OnResetClicked();
  void OnSaveMeshClicked(QString filename);
  void OnSavePoseClicked(QString filename);

 private slots:

  void OnReadInput();

 private:

  // Data.
  RgbdInput* input_ = nullptr;
  RegularGridFusionPipeline* pipeline_ = nullptr;

  // GUI.
  ControlWidget* control_widget_ = nullptr;
  MainWidget* main_widget_ = nullptr;

  QTimer* read_input_timer_ = nullptr;

};

#endif  // MAIN_CONTROLLER_H
