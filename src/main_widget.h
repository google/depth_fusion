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
#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

#include <memory>

#include <GL/glew.h>
#include <QOpenGLWidget>

#include <core/cameras/PerspectiveCamera.h>
#include <core/common/Array2D.h>
#include <core/common/ArrayUtils.h>
#include <core/common/BasicTypes.h>
#include <qt_interop/FPSControls.h>

#include "gl_state.h"
#include "static_multi_camera_gl_state.h"
#include "rgbd_camera_parameters.h"

struct InputBuffer;
class RegularGridFusionPipeline;
class StaticMultiCameraPipeline;

class MainWidget : public QOpenGLWidget {
 public:

  MainWidget(const RGBDCameraParameters& camera_params,
    QWidget* parent = nullptr);

  void SetPipeline(RegularGridFusionPipeline* pipeline);
  void SetPipeline(StaticMultiCameraPipeline* pipeline);

  GLState* GetSingleCameraGLState() const;
  StaticMultiCameraGLState* GetSMCGLState() const;

 protected:
  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL(int w, int h);

  virtual void mousePressEvent(QMouseEvent* e) override;
  virtual void mouseMoveEvent(QMouseEvent* e) override;
  virtual void mouseReleaseEvent(QMouseEvent* e) override;

  virtual void keyPressEvent(QKeyEvent* e) override;

 private:

  RegularGridFusionPipeline* pipeline_ = nullptr;
  std::unique_ptr<GLState> gl_state_ = nullptr;

  StaticMultiCameraPipeline* smc_pipeline_ = nullptr;
  std::unique_ptr<StaticMultiCameraGLState> smc_gl_state_ = nullptr;

  FPSControls fps_controls_;
  PerspectiveCamera free_camera_;
  const RGBDCameraParameters camera_params_;
};

#endif  // MAIN_WIDGET_H
