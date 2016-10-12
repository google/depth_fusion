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
#pragma once
// TODO: use include guard

#include <memory>
#include <unordered_map>

#include <core/common/Array2D.h>

#include <cuda/DeviceArray2D.h>
#include <cuda/gl/Texture2D.h>

#include <GL/common/GLProgramPipeline.h>
#include <GL/common/GLSamplerObject.h>
#include <GL/common/GLSeparableProgram.h>
#include <GL/GL_45/GLTexture2D.h>
#include <GL/GL_45/drawables/Axes.h>
#include <GL/GL_45/drawables/Frustum.h>
#include <GL/GL_45/drawables/PointCloud.h>
#include <GL/GL_45/drawables/TexturedRectangle.h>
#include <GL/GL_45/drawables/WireframeBox.h>

class RegularGridFusionPipeline;

// TODO: rename this struct
struct RemappedTexture {
  GLTexture2D* texture;
  Vector2f size_scale;
  Matrix4f color_transform;
};

class GLState {
 public:
  // 11 inches x 8.5 inches is 0.2794 meters x 0.2159 meters.
  const Rect2f kGridBoard{ { -0.04225f, -0.015f },{ 0.2794f, 0.2159f } };

  GLState(RegularGridFusionPipeline* pipeline);

  void NotifyTSDFUpdated();
  void Resize(const Vector2i& size);
  void Render(const PerspectiveCamera& free_camera);

 private:

  // ----- Helper functions -----
  void LoadShaders();

  void DrawWorldAxes();
  void DrawColorTrackingBoard();
  void DrawUnprojectedPointCloud();
  void DrawFullscreenRaycast();
  void DrawCameraFrustaAndTSDFGrid();
  void DrawInputsAndIntermediates();
  void DrawRemappedTextures(const std::vector<RemappedTexture>& textures);

  // ----- State -----
  RegularGridFusionPipeline* pipeline_ = nullptr;
  bool tsdf_is_dirty_ = true;
  PerspectiveCamera free_camera_;

  // ----- Drawables -----
  Axes world_axes_;
  TexturedRectangle input_buffer_textured_rect_;
  TexturedRectangle board_rectangle_;
  GLTexture2D board_texture_;
  Frustum tracked_rgb_camera_;
  Frustum tracked_depth_camera_;
  WireframeBox tsdf_bbox_;
  PointCloud xy_coords_;

  GLTexture2D color_texture_;
  GLTexture2D color_tracking_vis_texture_;

  // TODO: can interop depth_texture. Then, can use undistorted version.
  GLTexture2D depth_texture_;  // Float depth in meters.
  libcgt::cuda::gl::Texture2D smoothed_depth_tex_;
  libcgt::cuda::gl::Texture2D smoothed_incoming_normals_tex_;
  libcgt::cuda::gl::Texture2D pose_estimation_vis_tex_;

  // Raycasting from the current viewpoint.
  libcgt::cuda::gl::Texture2D raycasted_normals_tex_;

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