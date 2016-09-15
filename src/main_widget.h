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
#include <QGLWidget>

#include <core/cameras/PerspectiveCamera.h>
#include <core/common/Array2D.h>
#include <core/common/ArrayUtils.h>
#include <core/common/BasicTypes.h>

#define GL_PLATFORM_45
#include <GL/common/GLProgramManager.h>
#include <GL/common/GLProgramPipeline.h>
#include <GL/common/GLSamplerObject.h>
#include <GL/GL_45/GLTexture2D.h>
#include <GL/GL_45/drawables/Axes.h>
#include <GL/GL_45/drawables/Frustum.h>
#include <GL/GL_45/drawables/PointCloud.h>
#include <GL/GL_45/drawables/TexturedRectangle.h>
#include <GL/GL_45/drawables/WireframeBox.h>

#include <qt_interop/FPSControls.h>

struct InputBuffer;
class RegularGridFusionPipeline;

#include "rgbd_camera_parameters.h"

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

struct GLState {

    // 11 inches x 8.5 inches is 0.2794 meters x 0.2159 meters.
    const Rect2f kGridBoard{ { -0.04225f, -0.015f }, { 0.2794f, 0.2159f } };

    GLState(
      const Vector2i& board_resolution,
      const Vector2i& color_resolution,
      const Vector2i& depth_resolution) :
        // HACK
        board_rectangle_(kGridBoard),
        board_texture_(board_resolution, GLImageInternalFormat::R8),
        color_texture_(color_resolution, GLImageInternalFormat::RGB8),
        color_tracking_vis_(color_resolution),
        color_tracking_vis_texture_(color_resolution,
          GLImageInternalFormat::RGB8),
        depth_texture_(depth_resolution, GLImageInternalFormat::R32F),
        depth_vis_(depth_resolution),
        depth_vis_texture_(depth_resolution, GLImageInternalFormat::R8),
        smoothed_depth_vis_texture_(depth_resolution, GLImageInternalFormat::R8),
        normal_vis_texture_(depth_resolution, GLImageInternalFormat::RGB8),
        compatibility_map_texture_(depth_resolution, GLImageInternalFormat::RGB8),
        xy_coords_(2, depth_resolution.x * depth_resolution.y),
        raycasted_points_(4, depth_resolution.x * depth_resolution.y),
        raycasted_normals_tex_(depth_resolution,
          GLImageInternalFormat::RGBA32F)
    {
      programs_.addFromSourceCode("drawColorVS",
        GLSeparableProgram::Type::VERTEX_SHADER, kDrawColorVSSrc);
      programs_.addFromSourceCode("positionOnlyVS",
        GLSeparableProgram::Type::VERTEX_SHADER, kPositionOnlyVSSrc);
      programs_.addFromSourceCode("drawTextureVS",
        GLSeparableProgram::Type::VERTEX_SHADER, kDrawTextureVSSrc);
      programs_.addFromSourceCode("unprojectPointCloudVS",
        GLSeparableProgram::Type::VERTEX_SHADER, kUnprojectPointCloudVSSrc);

      programs_.addFromSourceCode("drawColorFS",
        GLSeparableProgram::Type::FRAGMENT_SHADER, kDrawColorFSSrc);
      programs_.addFromSourceCode("drawColorDiscardTransparentFS",
        GLSeparableProgram::Type::FRAGMENT_SHADER,
        kDrawColorDiscardTransparentFSSrc);
      programs_.addFromSourceCode("drawSingleColorFS",
        GLSeparableProgram::Type::FRAGMENT_SHADER, kDrawSingleColorFSSrc);
      programs_.addFromSourceCode("drawTextureFS",
        GLSeparableProgram::Type::FRAGMENT_SHADER, kDrawTextureFSSrc);

      draw_color_.attachProgram(programs_.get("drawColorVS"));
      draw_color_.attachProgram(programs_.get("drawColorFS"));

      draw_single_color_.attachProgram(programs_.get("positionOnlyVS"),
        GLProgramPipeline::Stage::VERTEX_SHADER_BIT);
      draw_single_color_.attachProgram(programs_.get("drawSingleColorFS"),
        GLProgramPipeline::Stage::FRAGMENT_SHADER_BIT);

      draw_texture_.attachProgram(programs_.get("drawTextureVS"));
      draw_texture_.attachProgram(programs_.get("drawTextureFS"));

      unproject_point_cloud_.attachProgram(
        programs_.get("unprojectPointCloudVS"));
      unproject_point_cloud_.attachProgram(
        programs_.get("drawColorDiscardTransparentFS"));

      GLTexture::SwizzleTarget swizzle_rrr1[4] =
      {
        GLTexture::SwizzleTarget::RED,
        GLTexture::SwizzleTarget::RED,
        GLTexture::SwizzleTarget::RED,
        GLTexture::SwizzleTarget::ONE
      };
      board_texture_.setSwizzleRGBA(swizzle_rrr1);
      depth_vis_texture_.setSwizzleRGBA(swizzle_rrr1);
      smoothed_depth_vis_texture_.setSwizzleRGBA(swizzle_rrr1);

      nearest_sampler_.setMinMagFilterModes(GLTextureFilterMode::NEAREST);
      nearest_sampler_.setWrapModes(GLWrapMode::CLAMP_TO_EDGE);

      linear_sampler_.setMinMagFilterModes(GLTextureFilterMode::LINEAR);
      linear_sampler_.setWrapModes(GLWrapMode::CLAMP_TO_EDGE);
    }

    // TODO(jiawen): refactor GLProgramManager: it might not be necessary.
    // Instead, pipelines should reference count programs so it's easy to set
    // stuff.
    GLProgramManager programs_;

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

    // float depth in meters, just for unprojecting.
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

class MainWidget : public QGLWidget {
 public:

   MainWidget(const RGBDCameraParameters& camera_params,
    const QGLFormat& format,
    QWidget* parent = nullptr);

  // HACK: properly hide these;
  Array2D<uint8_t> gl_board_image_;
  RegularGridFusionPipeline* pipeline_ = nullptr;
  InputBuffer* input_buffer_;

 protected:
  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL(int w, int h);

  virtual void mousePressEvent(QMouseEvent* e) override;
  virtual void mouseMoveEvent(QMouseEvent* e) override;
  virtual void mouseReleaseEvent(QMouseEvent* e) override;

  virtual void keyPressEvent(QKeyEvent* e) override;

 private:

  void UpdateCameraFrusta();

  void DrawWorldAxes();
  void DrawBoardTexture();
  void DrawCameraFrustaAndTSDFGrid();
  void DrawUnprojectedPointCloud();

  std::unique_ptr<GLState> gl_state_;

  FPSControls fps_controls_;
  PerspectiveCamera free_camera_;
  const RGBDCameraParameters camera_params_;
};

#endif  // MAIN_WIDGET_H
