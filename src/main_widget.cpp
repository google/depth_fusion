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
#include "main_widget.h"

#include <GL/glew.h>

#include <core/common/Array2DView.h>
#include <core/common/ArrayUtils.h>
#include <core/geometry/RectangleUtils.h>
#include <core/imageproc/ColorMap.h>

#include <GL/common/GLUtilities.h>

#include "input_buffer.h"
#include "regular_grid_fusion_pipeline.h"

const float kFreeCameraFovYRadians = 0.87f; // 50 degrees.
const float kFreeCameraZNear = 0.001f;
const float kFreeCameraZFar = 100.0f;

using libcgt::core::arrayutils::cast;
using libcgt::core::cameras::GLFrustum;
using libcgt::core::cameras::Intrinsics;
using libcgt::core::imageproc::linearRemapToLuminance;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::inverse;
using libcgt::core::geometry::translate;

// TODO: get rid of camera_params.
MainWidget::MainWidget(const RGBDCameraParameters& camera_params,
  QWidget* parent) :
  QOpenGLWidget(parent),
  camera_params_(camera_params) {
  QSurfaceFormat format = QSurfaceFormat::defaultFormat();
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setMajorVersion(4);
  format.setMinorVersion(5);
//#if _DEBUG
#if 1
  format.setOption(QSurfaceFormat::DebugContext, true);
#endif
  setFormat(format);

  const float aspectRatio = static_cast<float>(width()) / height();
  free_camera_.setFrustum(GLFrustum::makeSymmetricPerspective(
    kFreeCameraFovYRadians, aspectRatio, kFreeCameraZNear, kFreeCameraZFar));
}

void MainWidget::SetPipeline(RegularGridFusionPipeline* pipeline) {
  pipeline_ = pipeline;
}

void MainWidget::SetPipeline(StaticMultiCameraPipeline* pipeline) {
  smc_pipeline_ = pipeline;
}

GLState* MainWidget::GetSingleCameraGLState() const {
  return gl_state_.get();
}

StaticMultiCameraGLState* MainWidget::GetSMCGLState() const {
  return smc_gl_state_.get();
}

void MainWidget::initializeGL() {
  GLenum err = glewInit();
  if (err != GLEW_OK) {
    fprintf(stderr, "Error initializing GLEW: %s\n", glewGetErrorString(err));
    exit(1);
  }
  glClearColor(0, 0, 0, 0);
  glEnable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBlendEquation(GL_FUNC_ADD);

  if (pipeline_ != nullptr) {
    gl_state_ = std::make_unique<GLState>(pipeline_);
  }

  if (smc_pipeline_ != nullptr) {
    smc_gl_state_ = std::make_unique<StaticMultiCameraGLState>(smc_pipeline_);
  }
}

// virtual
void MainWidget::keyPressEvent(QKeyEvent* e) {

}

void MainWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (smc_gl_state_ != nullptr) {
      smc_gl_state_->Render(free_camera_);
    }

    if (gl_state_ != nullptr) {
      gl_state_->Render(free_camera_);
    }
}

void MainWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);

    float aspectRatio = static_cast<float>(w) / h;
    free_camera_.setFrustum(GLFrustum::makeSymmetricPerspective(
      kFreeCameraFovYRadians, aspectRatio, kFreeCameraZNear, kFreeCameraZFar));

    if (gl_state_ != nullptr) {
      gl_state_->Resize({w, h});
    }

    if(smc_gl_state_ != nullptr) {
      smc_gl_state_->Resize({w, h});
    }

    update();
}

void MainWidget::mousePressEvent(QMouseEvent* e) {
    fps_controls_.handleMousePressEvent(e);
    update();
}

void MainWidget::mouseMoveEvent(QMouseEvent* e) {
    fps_controls_.handleMouseMoveEvent(e, free_camera_);
    update();
}

void MainWidget::mouseReleaseEvent(QMouseEvent* e) {
    fps_controls_.handleMouseReleaseEvent(e);
    update();
}
