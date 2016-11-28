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
#include "static_multi_camera_gl_state.h"

#include <core/common/ForND.h>
#include <core/geometry/RectangleUtils.h>

#include "static_multi_camera_pipeline.h"

using libcgt::core::for2D;
using libcgt::core::geometry::translate;
using libcgt::cuda::gl::Texture2D;

namespace {
const std::string kDrawTextureVSSrc =
#include "shaders/draw_texture.vs.glsl"
;
const std::string kDrawColorVSSrc =
#include "shaders/draw_color.vs.glsl"
;
const std::string kPositionOnlyVSSrc =
#include "shaders/position_only.vs.glsl"
;
const std::string kDrawColorFSSrc =
#include "shaders/draw_color.fs.glsl"
;
const std::string kUnprojectPointCloudVSSrc =
#include "shaders/unproject_point_cloud.vs.glsl"
;
const std::string kDrawTextureFSSrc =
#include "shaders/draw_texture.fs.glsl"
;
const std::string kDrawColorDiscardTransparentFSSrc =
#include "shaders/draw_color_discard_transparent.fs.glsl"
;
const std::string kDrawSingleColorFSSrc =
#include "shaders/draw_single_color.fs.glsl"
;
}  // namespace

// TODO: shaders can be shared GL state

const bool kDrawUnprojectedPointCloud = false;
const bool kDrawFullscreenRaycast = true;
const int kFullscreenRaycastDownsampleFactor = 1;

StaticMultiCameraGLState::StaticMultiCameraGLState(
  StaticMultiCameraPipeline* pipeline, QOpenGLWidget* parent) :
  parent_(parent),
  pipeline_(pipeline),
  depth_camera_frusta_(pipeline->NumCameras()),
  free_camera_world_positions_tex_(
    GLTexture2D(pipeline->GetCameraParameters(0).depth.resolution,
      GLImageInternalFormat::RGBA32F),
    Texture2D::MapFlags::WRITE_DISCARD),
  free_camera_world_normals_tex_(
    GLTexture2D(pipeline->GetCameraParameters(0).depth.resolution,
      GLImageInternalFormat::RGBA32F),
    Texture2D::MapFlags::WRITE_DISCARD),
  xy_coords_(2,
    pipeline->GetCameraParameters(0).depth.resolution.x *
    pipeline->GetCameraParameters(0).depth.resolution.y) {
	programs_.emplace("drawColorVS",
		GLSeparableProgram(GLSeparableProgram::Type::VERTEX_SHADER,
			kDrawColorVSSrc.c_str()));
	programs_.emplace("positionOnlyVS",
		GLSeparableProgram(GLSeparableProgram::Type::VERTEX_SHADER,
			kPositionOnlyVSSrc.c_str()));
	programs_.emplace("drawTextureVS",
		GLSeparableProgram(GLSeparableProgram::Type::VERTEX_SHADER,
			kDrawTextureVSSrc.c_str()));
	programs_.emplace("unprojectPointCloudVS",
		GLSeparableProgram(GLSeparableProgram::Type::VERTEX_SHADER,
			kUnprojectPointCloudVSSrc.c_str()));

	programs_.emplace("drawColorFS",
		GLSeparableProgram(GLSeparableProgram::Type::FRAGMENT_SHADER,
			kDrawColorFSSrc.c_str()));
	programs_.emplace("drawColorDiscardTransparentFS",
		GLSeparableProgram(GLSeparableProgram::Type::FRAGMENT_SHADER,
			kDrawColorDiscardTransparentFSSrc.c_str()));
	programs_.emplace("drawSingleColorFS",
		GLSeparableProgram(GLSeparableProgram::Type::FRAGMENT_SHADER,
			kDrawSingleColorFSSrc.c_str()));
	programs_.emplace("drawTextureFS",
		GLSeparableProgram(GLSeparableProgram::Type::FRAGMENT_SHADER,
			kDrawTextureFSSrc.c_str()));

	draw_color_.attachProgram(programs_["drawColorVS"]);
	draw_color_.attachProgram(programs_["drawColorFS"]);

	draw_single_color_.attachProgram(programs_["positionOnlyVS"]);
	draw_single_color_.attachProgram(programs_["drawSingleColorFS"]);

	draw_texture_.attachProgram(programs_["drawTextureVS"]);
	draw_texture_.attachProgram(programs_["drawTextureFS"]);

	unproject_point_cloud_.attachProgram(programs_["unprojectPointCloudVS"]);
	unproject_point_cloud_.attachProgram(
		programs_["drawColorDiscardTransparentFS"]);

	GLTexture::SwizzleTarget swizzle_rrr1[4] =
	{
		GLTexture::SwizzleTarget::RED,
		GLTexture::SwizzleTarget::RED,
		GLTexture::SwizzleTarget::RED,
		GLTexture::SwizzleTarget::ONE
	};

  Vector2i depth_resolution =
    pipeline->GetCameraParameters(0).depth.resolution;
  for (int i = 0; i < pipeline_->NumCameras(); ++i) {
    raw_depth_textures_.emplace_back(
      Texture2D(GLTexture2D(depth_resolution, GLImageInternalFormat::R32F),
        Texture2D::MapFlags::WRITE_DISCARD));
    undistorted_depth_textures_.emplace_back(
      Texture2D(GLTexture2D(depth_resolution, GLImageInternalFormat::R32F),
        Texture2D::MapFlags::WRITE_DISCARD));
    raw_depth_textures_.back().texture().setSwizzleRGBA(swizzle_rrr1);
    undistorted_depth_textures_.back().texture().setSwizzleRGBA(swizzle_rrr1);
  }

  depth_camera_frusta_[0].updateColor({1, 0, 0, 1});
  if (pipeline_->NumCameras() > 1) {
    depth_camera_frusta_[1].updateColor({0, 1, 0, 1});
  }
  if (pipeline_->NumCameras() > 2) {
    depth_camera_frusta_[2].updateColor({0, 0, 1, 1});
  }

  tsdf_bbox_.updatePositions(pipeline_->TSDFGridBoundingBox(),
    pipeline_->TSDFWorldFromGridTransform().asMatrix());

  // Initialize xy_coords_.
  {
    auto mb = xy_coords_.mapAttribute<Vector2f>(0);
    Array2DWriteView<Vector2f> points2D(mb.view().pointer(), depth_resolution);
    for2D(depth_resolution, [&](const Vector2i& xy) {
        points2D[xy] = Vector2f{xy.x + 0.5f, xy.y + 0.5f};
    });
  }

	nearest_sampler_.setMinMagFilterModes(GLTextureFilterMode::NEAREST);
	nearest_sampler_.setWrapModes(GLWrapMode::CLAMP_TO_EDGE);

	linear_sampler_.setMinMagFilterModes(GLTextureFilterMode::LINEAR);
	linear_sampler_.setWrapModes(GLWrapMode::CLAMP_TO_EDGE);
}

void StaticMultiCameraGLState::NotifyTSDFUpdated() {
  tsdf_is_dirty_ = true;
}

void StaticMultiCameraGLState::Resize(const Vector2i& size) {
  window_size_ = size;
  Vector2i downsampled_size = size / kFullscreenRaycastDownsampleFactor;
  free_camera_world_positions_.resize(downsampled_size);
  free_camera_world_normals_.resize(downsampled_size);

  free_camera_world_positions_tex_ =
    libcgt::cuda::gl::Texture2D(
      GLTexture2D(downsampled_size, GLImageInternalFormat::RGBA32F),
      libcgt::cuda::gl::Texture2D::MapFlags::WRITE_DISCARD
    );
  free_camera_world_normals_tex_ =
    libcgt::cuda::gl::Texture2D(
      GLTexture2D(downsampled_size, GLImageInternalFormat::RGBA32F),
      libcgt::cuda::gl::Texture2D::MapFlags::WRITE_DISCARD
    );
}

void StaticMultiCameraGLState::Render(const PerspectiveCamera& free_camera) {
  if (free_camera != free_camera_) {
    free_camera_ = free_camera;
    tsdf_is_dirty_ = true;
  }

  // TODO: notify that input has changed
  for(size_t i = 0; i < raw_depth_textures_.size(); ++i) {
    raw_depth_textures_[i].texture().set(
      pipeline_->GetInputBuffer(i).depth_meters);

    {
      auto mr = undistorted_depth_textures_[i].map();
      pipeline_->GetUndistortedDepthMap(i).copyToArray(mr.array());
    }
  }

  // TODO: only when positions have changed, which in this case, is never.
  // UpdateCameraFrusta()
  for(size_t i = 0; i < depth_camera_frusta_.size(); ++i) {
    depth_camera_frusta_[i].updatePositions(pipeline_->GetDepthCamera(i));
  }

  DrawWorldAxes();
  DrawCameraFrustaAndTSDFGrid();
  if (kDrawUnprojectedPointCloud) {
    DrawUnprojectedPointClouds();
  }
  if (kDrawFullscreenRaycast) {
    DrawFullscreenRaycast();
  }
  DrawInputsAndIntermediates();
}

#include <core/math/MathUtils.h>
#include <core/geometry/RangeUtils.h>
#include <GLUtilities.h>
#include <vecmath/Vector4f.h>

// TODO: make a draw textures across accumulator.
void StaticMultiCameraGLState::DrawInputsAndIntermediates() {
  glDisable(GL_DEPTH_TEST);
  Rect2i vp = GLUtilities::getViewport();

  float scale;
  float offset;
  libcgt::core::geometry::rescaleRangeToScaleOffset(0.2f, 10.0f, 0, 1,
    scale, offset);
  Matrix4f colorMatrix = Matrix4f::uniformScaling(scale) *
    Matrix4f::translation(Vector3f{offset});
  colorMatrix.setRow(3, Vector4f{1, 0, 0, 0});

  GLSeparableProgram& vs = programs_["drawTextureVS"];
  vs.setUniformMatrix4f(0, Matrix4f::identity());
  GLSeparableProgram& fs = programs_["drawTextureFS"];
  fs.setUniformInt(0, 0); // sampler
  fs.setUniformMatrix4f(1, colorMatrix);
  draw_texture_.bind();

  Vector2i sz = raw_depth_textures_[0].texture().size() / 2;
  Rect2i current_rect{{ 0, 0 }, sz};

  for (size_t i = 0; i < raw_depth_textures_.size(); ++i) {
    GLUtilities::setViewport(current_rect);
    raw_depth_textures_[i].texture().bind(0);
    input_buffer_textured_rect_.draw();
    current_rect = translate(current_rect, { sz.x, 0 });

    GLUtilities::setViewport(current_rect);
    undistorted_depth_textures_[i].texture().bind(0);
    input_buffer_textured_rect_.draw();
    current_rect = translate(current_rect, { sz.x, 0 });
  }

  GLUtilities::setViewport(vp);
  glEnable(GL_DEPTH_TEST);
}

void StaticMultiCameraGLState::DrawWorldAxes() {
  GLSeparableProgram& vs = programs_["drawColorVS"];
  vs.setUniformMatrix4f(0, free_camera_.viewProjectionMatrix());
  draw_color_.bind();

  world_axes_.draw();

  GLProgramPipeline::unbindAll();
}

void StaticMultiCameraGLState::DrawCameraFrustaAndTSDFGrid() {
  GLSeparableProgram& vs = programs_["drawColorVS"];
  vs.setUniformMatrix4f(0, free_camera_.viewProjectionMatrix());
  draw_color_.bind();

  for(size_t i = 0; i < depth_camera_frusta_.size(); ++i) {
    depth_camera_frusta_[i].draw();
  }
  tsdf_bbox_.draw();

  GLProgramPipeline::unbindAll();
}

// HACK
namespace {
Matrix4f normalsToRGBA()
{
  Matrix4f m = Matrix4f::uniformScaling( 0.5f ) *
    Matrix4f::translation(Vector3f{1.0f});
  m.setRow(3, Vector4f{1, 0, 0, 0});
  return m;
}
}

void StaticMultiCameraGLState::DrawUnprojectedPointClouds() {
  const int kDepthTextureUnit = 0;

  unproject_point_cloud_.bind();
  GLSeparableProgram& vs = programs_["unprojectPointCloudVS"];
  vs.setUniformMatrix4f(0, free_camera_.viewProjectionMatrix());

  for (int i = 0; i < pipeline_->NumCameras(); ++i) {
  //for(int i = 0; i < 1; ++i) {
    auto params = pipeline_->GetCameraParameters(i).depth;
    Vector4f flpp{
      params.intrinsics.focalLength, params.intrinsics.principalPoint};
    Vector2f depth_range{
      params.depth_range.minimum(), params.depth_range.maximum() };
    vs.setUniformVector4f(1, flpp);
    vs.setUniformVector2f(2, depth_range);
    vs.setUniformMatrix4f(3,
      pipeline_->GetDepthCamera(i).worldFromCamera().asMatrix());

    vs.setUniformInt(4, kDepthTextureUnit);
    undistorted_depth_textures_[i].texture().bind(kDepthTextureUnit);
    nearest_sampler_.bind(kDepthTextureUnit);

    if (i == 0) {
      vs.setUniformVector4f(5, Vector4f(1, 0, 0, 1));
    } else if(i == 1) {
      vs.setUniformVector4f(5, Vector4f(0, 1, 0, 1));
    } else if(i == 2) {
      vs.setUniformVector4f(5, Vector4f(0, 0, 1, 1));
    } else {
      vs.setUniformVector4f(5, Vector4f{1});
    }

    xy_coords_.draw();
  }
}

void StaticMultiCameraGLState::DrawFullscreenRaycast() {
  // Update the buffer.
  if (tsdf_is_dirty_) {
    pipeline_->Raycast(free_camera_,
      free_camera_world_positions_, free_camera_world_normals_ );
    {
      auto mr = free_camera_world_positions_tex_.map();
      free_camera_world_positions_.copyToArray(mr.array());
    }
    {
      auto mr = free_camera_world_normals_tex_.map();
      free_camera_world_normals_.copyToArray(mr.array());
    }
    tsdf_is_dirty_ = false;
  }

  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);

  GLSeparableProgram& vs = programs_["drawTextureVS"];
  vs.setUniformMatrix4f(0, Matrix4f::identity());
  GLSeparableProgram& fs = programs_["drawTextureFS"];
  fs.setUniformInt(0, 0); // sampler
  fs.setUniformMatrix4f(1, normalsToRGBA());
  draw_texture_.bind();

  free_camera_world_normals_tex_.texture().bind(0);
  input_buffer_textured_rect_.draw();

  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
}
