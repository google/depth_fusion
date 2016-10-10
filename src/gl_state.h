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

#include <unordered_map>

#include <core/common/Array2D.h>

#include <GL/common/GLProgramPipeline.h>
#include <GL/common/GLSamplerObject.h>
#include <GL/common/GLSeparableProgram.h>
#include <GL/GL_45/GLTexture2D.h>
#include <GL/GL_45/drawables/Axes.h>
#include <GL/GL_45/drawables/Frustum.h>
#include <GL/GL_45/drawables/PointCloud.h>
#include <GL/GL_45/drawables/TexturedRectangle.h>
#include <GL/GL_45/drawables/WireframeBox.h>

struct GLState {
  // 11 inches x 8.5 inches is 0.2794 meters x 0.2159 meters.
  const Rect2f kGridBoard{ { -0.04225f, -0.015f },{ 0.2794f, 0.2159f } };
  
  GLState(
	const Vector2i& board_resolution,
	const Vector2i& color_resolution,
	const Vector2i& depth_resolution);
  std::unordered_map<std::string, GLSeparableProgram> programs_;
  
  GLProgramPipeline draw_color_;
  GLProgramPipeline draw_single_color_;
  GLProgramPipeline draw_texture_;
  GLProgramPipeline unproject_point_cloud_;
  
  Axes world_axes_;
  
  TexturedRectangle board_rectangle_;
  GLTexture2D board_texture_;
  
  GLTexture2D color_texture_;

  Array2D<uint8x3> color_tracking_vis_;
  GLTexture2D color_tracking_vis_texture_;
  
  // Float depth in meters, just for unprojecting.
  GLTexture2D depth_texture_;
  // Luma visualization of raw depth.
  Array2D<uint8_t> depth_vis_;
  GLTexture2D depth_vis_texture_;
  
  // Luma visualization of smoothed depth.
  GLTexture2D smoothed_depth_vis_texture_;
  GLTexture2D normal_vis_texture_;
  
  GLTexture2D compatibility_map_texture_;
  
  Frustum tracked_rgb_camera_;
  TexturedRectangle tracked_rgb_camera_image_rect_;
  
  Frustum tracked_depth_camera_;
  TexturedRectangle tracked_depth_camera_image_rect_;
  
  PointCloud xy_coords_;
  WireframeBox tsdf_bbox_;
  
  PointCloud raycasted_points_;
  TexturedRectangle raycasted_normals_rect_;
  GLTexture2D raycasted_normals_tex_;
  
  GLSamplerObject nearest_sampler_;
  GLSamplerObject linear_sampler_;
};
