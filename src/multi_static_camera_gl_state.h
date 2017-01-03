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
#ifndef MULTI_STATIC_CAMERA_GL_STATE_H
#define MULTI_STATIC_CAMERA_GL_STATE_H

#include <unordered_map>

#include <GL/glew.h>
#include <QObject>
#include <QOpenGLWidget>

#include "libcgt/core/common/Array2D.h"
#include "libcgt/cuda/DeviceArray2D.h"
#include "libcgt/cuda/gl/Texture2D.h"
#include "libcgt/GL/GLProgramPipeline.h"
#include "libcgt/GL/GLSamplerObject.h"
#include "libcgt/GL/GLSeparableProgram.h"
#include "libcgt/GL/GL_45/GLTexture2D.h"
#include "libcgt/GL/GL_45/drawables/Axes.h"
#include "libcgt/GL/GL_45/drawables/Frustum.h"
#include "libcgt/GL/GL_45/drawables/PointCloud.h"
#include "libcgt/GL/GL_45/drawables/TexturedRectangle.h"
#include "libcgt/GL/GL_45/drawables/WireframeBox.h"

class MultiStaticCameraPipeline;

class MultiStaticCameraGLState : public QObject {

 Q_OBJECT

 public:
  MultiStaticCameraGLState(MultiStaticCameraPipeline* pipeline,
    QOpenGLWidget* parent);

  //void NotifyInputUpdated();
  void NotifyTSDFUpdated();

  void Resize(const Vector2i& size);
  void Render(const PerspectiveCamera& free_camera);

 private:

  // ----- Helper functions -----
  void DrawInputsAndIntermediates();
  void DrawWorldAxes();
  void DrawCameraFrustaAndTSDFGrid();
  void DrawUnprojectedPointClouds();
  void DrawFullscreenRaycast();

  // ----- State -----
  QOpenGLWidget* parent_ = nullptr;
  MultiStaticCameraPipeline* pipeline_ = nullptr;
  bool tsdf_is_dirty_ = true;
  PerspectiveCamera free_camera_;
  Vector2i window_size_;

  // ----- Drawables -----
  Axes world_axes_;
  TexturedRectangle input_buffer_textured_rect_;
  std::vector<Frustum> depth_camera_frusta_;
  WireframeBox tsdf_bbox_;
  PointCloud xy_coords_;

  // TODO: interop, avoid copy:
  //   need to wrap a DeviceOpaqueArray, and then enable surface writes.
  std::vector<libcgt::cuda::gl::Texture2D> raw_depth_textures_;
  std::vector<libcgt::cuda::gl::Texture2D> undistorted_depth_textures_;

  // Raycasting from the current camera pose, resized as the view changes.
  DeviceArray2D<float4> free_camera_world_positions_;
  DeviceArray2D<float4> free_camera_world_normals_;
  libcgt::cuda::gl::Texture2D free_camera_world_positions_tex_;
  libcgt::cuda::gl::Texture2D free_camera_world_normals_tex_;

  // ----- GL helper objects -----
  std::unordered_map<std::string, GLSeparableProgram> programs_;

  GLProgramPipeline draw_color_;
  GLProgramPipeline draw_single_color_;
  GLProgramPipeline draw_texture_;
  GLProgramPipeline unproject_point_cloud_;

  GLSamplerObject nearest_sampler_;
  GLSamplerObject linear_sampler_;
};

#endif  // MULTI_STATIC_CAMERA_GL_STATE_H
