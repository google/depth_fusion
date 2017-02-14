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
#include <vector>

#include <GL/glew.h>
#include <QOpenGLWidget>

#include "libcgt/core/cameras/PerspectiveCamera.h"
#include "libcgt/core/common/Array2D.h"
#include "libcgt/core/common/ArrayUtils.h"
#include "libcgt/core/common/BasicTypes.h"
#include "libcgt/core/vecmath/EuclideanTransform.h"
#include "libcgt/GL/GLProgramPipeline.h"
#include "libcgt/GL/GL_45/drawables/Axes.h"
#include "libcgt/GL/GL_45/drawables/Frustum.h"
#include "libcgt/qt_interop/FPSControls.h"

#include "../rgbd_camera_parameters.h"

class MainWidget : public QOpenGLWidget
{
public:

  typedef std::vector<Frustum> FrustumPath;
  using EuclideanTransform = libcgt::core::vecmath::EuclideanTransform;

  MainWidget(QWidget* parent = nullptr);

  void addCameraPath(const std::vector<EuclideanTransform>& poses,
    const CameraParameters& intrinsics,
    const Vector4f& color0, const Vector4f& color1);

protected:
  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL(int w, int h);

  virtual void mousePressEvent(QMouseEvent* e) override;
  virtual void mouseMoveEvent(QMouseEvent* e) override;
  virtual void mouseReleaseEvent(QMouseEvent* e) override;

  virtual void keyPressEvent(QKeyEvent* e) override;

private:

  int selected_path_ = -1;
  std::vector<FrustumPath> paths_;
  std::vector<int> sampling_rate_;

  FPSControls fps_controls_;
  PerspectiveCamera free_camera_;

  std::unique_ptr<GLProgramPipeline> draw_color_;
  std::unique_ptr<GLProgramPipeline> draw_single_color_;

  std::unique_ptr<Axes> world_axes_;
};

#endif  // MAIN_WIDGET_H
