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

#include "libcgt/core/common/ArrayView.h"
#include "libcgt/core/common/ArrayUtils.h"
#include "libcgt/core/geometry/RectangleUtils.h"
#include "libcgt/core/imageproc/ColorMap.h"
#include "libcgt/core/math/MathUtils.h"
#include "libcgt/GL/GLSeparableProgram.h"
#include "libcgt/GL/GLUtilities.h"

const float kFreeCameraFovYRadians = 0.87f; // 50 degrees.
const float kFreeCameraZNear = 0.001f;
const float kFreeCameraZFar = 100.0f;

using libcgt::core::arrayutils::cast;
using libcgt::core::cameras::GLFrustum;
using libcgt::core::cameras::Intrinsics;
using libcgt::core::imageproc::linearRemapToLuminance;
using libcgt::core::math::clamp;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::inverse;
using libcgt::core::geometry::translate;

namespace {
const std::string kPositionColorVSSrc =
#include "../shaders/position_color.vs.glsl"
;
const std::string kPositionOnlyVSSrc =
#include "../shaders/position_only.vs.glsl"
;
const std::string kDrawColorFSSrc =
#include "../shaders/draw_color.fs.glsl"
;
const std::string kDrawSingleColorFSSrc =
#include "../shaders/draw_single_color.fs.glsl"
;
}  // namespace

namespace {
  void DebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity,
    GLsizei length, const GLchar* message, const void* userParam) {
    if (severity != GL_DEBUG_SEVERITY_NOTIFICATION) {
      fprintf(stderr, "[GL DEBUG]: %s\n", message);
    }
  }
}

MainWidget::MainWidget(QWidget* parent) :
  QOpenGLWidget(parent) {
  QSurfaceFormat format = QSurfaceFormat::defaultFormat();
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setMajorVersion(4);
  format.setMinorVersion(5);
#if _DEBUG
  format.setOption(QSurfaceFormat::DebugContext, true);
#endif
  setFormat(format);

  const float aspectRatio = static_cast<float>(width()) / height();
  free_camera_.setFrustum(GLFrustum::makeSymmetricPerspective(
    kFreeCameraFovYRadians, aspectRatio, kFreeCameraZNear, kFreeCameraZFar));
}

void MainWidget::addCameraPath(const std::vector<EuclideanTransform>& poses,
    const CameraParameters& parameters,
    const Vector4f& color0, const Vector4f& color1) {
  makeCurrent();

  FrustumPath path(poses.size());
  for (int i = 0; i < poses.size(); ++i)
  {
    PerspectiveCamera camera;
    camera.setFrustum(parameters.intrinsics, Vector2f(parameters.resolution),
      0.02f, 0.1f );
    camera.setCameraFromWorld( poses[ i ] );

    float frac = static_cast< float >( i ) / ( poses.size() );
    path[i].updateColor(
      libcgt::core::math::lerp(color0, color1, frac));
    path[i].updatePositions(
      camera);
  }

  paths_.push_back(std::move(path));
  sampling_rate_.push_back(1);

  if( selected_path_ == -1 ) {
    selected_path_ = 0;
  }

  doneCurrent();
}

void MainWidget::initializeGL() {
  GLenum err = glewInit();
  if (err != GLEW_OK) {
    fprintf(stderr, "Error initializing GLEW: %s\n", glewGetErrorString(err));
    exit(1);
  }

#if _DEBUG
  glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
  glDebugMessageCallback(DebugCallback, this);
#endif

  glClearColor(0, 0, 0, 0);
  glEnable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBlendEquation(GL_FUNC_ADD);

  // Load shaders.
  auto positionColorVS = std::make_shared<GLSeparableProgram>(
    GLSeparableProgram::Type::VERTEX_SHADER, kPositionColorVSSrc.c_str());
  auto positionOnlyVS = std::make_shared<GLSeparableProgram>(
    GLSeparableProgram::Type::VERTEX_SHADER, kPositionOnlyVSSrc.c_str());
  auto drawColorFS = std::make_shared<GLSeparableProgram>(
    GLSeparableProgram::Type::FRAGMENT_SHADER, kDrawColorFSSrc.c_str());
  auto drawSingleColorFS = std::make_shared<GLSeparableProgram>(
    GLSeparableProgram::Type::FRAGMENT_SHADER, kDrawSingleColorFSSrc.c_str());

  draw_color_ = std::make_unique<GLProgramPipeline>();
  draw_color_->attachProgram(positionColorVS);
  draw_color_->attachProgram(drawColorFS);

  draw_single_color_ = std::make_unique<GLProgramPipeline>();
  draw_single_color_->attachProgram(positionOnlyVS);
  draw_single_color_->attachProgram(drawSingleColorFS);

  world_axes_ = std::make_unique<Axes>();
}

// virtual
void MainWidget::keyPressEvent(QKeyEvent* e) {
  switch (e->key()) {
  case Qt::Key_Up:
    if (selected_path_ != -1) {
      int i = selected_path_;
      int s = clamp(sampling_rate_[i] + 1, Range1i(1, paths_[i].size() / 3));
      sampling_rate_[i] = s;
      printf("path %d sampling rate is now: %d\n", i, s);
      update();
    }
    break;
  case Qt::Key_Down:
    if (selected_path_ != -1) {
      int i = selected_path_;
      int s = clamp(sampling_rate_[i] - 1, Range1i(1, paths_[i].size() / 3));
      sampling_rate_[i] = s;
      printf("path %d sampling rate is now: %d\n", i, s);
    }
    break;
  case Qt::Key_Left:
    selected_path_ = clamp(selected_path_ - 1, Range1i(paths_.size()));
    printf("selected path: %d\n", selected_path_);
    break;
  case Qt::Key_Right:
    selected_path_ = clamp(selected_path_ + 1, Range1i(paths_.size()));
    printf("selected path: %d\n", selected_path_);
    break;
  }
}

void MainWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Draw world axes.
    draw_color_->vertexProgram()->setUniformMatrix4f(0,
      free_camera_.viewProjectionMatrix());
    draw_color_->bind();
    world_axes_->draw();

    // Draw paths.
    for (size_t i = 0; i < paths_.size(); ++i) {
      auto& path = paths_[i];
      int sampling_rate = sampling_rate_[i];
      for (size_t j = 0; j < path.size(); j += sampling_rate) {
        path[j].draw();
      }
    }

    GLProgramPipeline::unbindAll();
}

void MainWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);

    float aspectRatio = static_cast<float>(w) / h;
    free_camera_.setFrustum(GLFrustum::makeSymmetricPerspective(
      kFreeCameraFovYRadians, aspectRatio, kFreeCameraZNear, kFreeCameraZFar));

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
