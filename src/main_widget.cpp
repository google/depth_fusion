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
using libcgt::core::arrayutils::copy;
using libcgt::core::cameras::GLFrustum;
using libcgt::core::cameras::Intrinsics;
using libcgt::core::imageproc::linearRemapToLuminance;
using libcgt::core::vecmath::EuclideanTransform;
using libcgt::core::vecmath::inverse;
using libcgt::core::geometry::rectangleutils::translate;

// TODO(jiawen): Consider making a visualizer instead of everything here.

MainWidget::MainWidget(const RGBDCameraParameters& camera_params,
  const QGLFormat& format, QWidget* parent) :
  QGLWidget(format, parent),
  camera_params_(camera_params) {
  const float aspectRatio = static_cast<float>(width()) / height();
  free_camera_.setFrustum(GLFrustum::makeSymmetricPerspective(
    kFreeCameraFovYRadians, aspectRatio, kFreeCameraZNear, kFreeCameraZFar));
}

void MainWidget::initializeGL() {
  // TODO(jiawen): Investigate (with a debug context) why there is an invalid
  // enum on startup.
  GLUtilities::printLastError();
  glClearColor(0, 0, 0, 0);
  glEnable(GL_DEPTH_TEST);

  gl_state_ = std::make_unique<GLState>(
    gl_board_image_.size(),
    camera_params_.color.resolution,
    camera_params_.depth.resolution
  );

  gl_state_->board_texture_.set(gl_board_image_);

  gl_state_->tracked_rgb_camera_.updateColor({ 1, 0, 0, 1 });
  gl_state_->tracked_depth_camera_.updateColor({ 0, 0, 1, 1 });

  gl_state_->tsdf_bbox_.updatePositions(
    pipeline_->regular_grid_.BoundingBox(),
    pipeline_->regular_grid_.WorldFromGrid().asMatrix());

  // Initialize gl_state_->xy_coords_.
  {
    auto mb = gl_state_->xy_coords_.mapAttribute<Vector2f>(0);
    Array2DView<Vector2f> points2D(mb.view().pointer(),
      camera_params_.depth.resolution);
    for (int y = 0; y < points2D.height(); ++y) {
      for (int x = 0; x < points2D.width(); ++x) {
        points2D[{x, y}] = Vector2f{ x + 0.5f, y + 0.5f };
      }
    }
  }
}

// virtual
void MainWidget::keyPressEvent(QKeyEvent* e) {

}

void MainWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (input_buffer_ != nullptr) {
      // TODO(jiawen): notify that it has a new input. Mark texture as dirty.
      gl_state_->color_texture_.set(input_buffer_->color_rgb);
      gl_state_->depth_texture_.set(input_buffer_->depth_meters);

      // TODO(jiawen): Ask the pipeline for a visualization of color tracking.
      copy(input_buffer_->color_rgb.readView(),
        gl_state_->color_tracking_vis_.writeView());
      gl_state_->color_tracking_vis_texture_.set(
        gl_state_->color_tracking_vis_);

      linearRemapToLuminance(
        input_buffer_->depth_meters, camera_params_.depth.depth_range,
        Range1f::fromMinMax(0.2f, 1.0f), gl_state_->depth_vis_);
      gl_state_->depth_vis_texture_.set(gl_state_->depth_vis_);

      gl_state_->compatibility_map_texture_.set(pipeline_->debug_icp_debug_vis_);
    }

    // TODO(jiawen): only if kf has new stuff...
    UpdateCameraFrusta();

    DrawWorldAxes();
    DrawBoardTexture();
    DrawCameraFrustaAndTSDFGrid();
    DrawUnprojectedPointCloud();

    // Draw the raytraced point cloud.
    if (true) {
      if (pipeline_->debug_points_world_.notNull()) {
        auto mb = gl_state_->raycasted_points_.mapAttribute<float4>(0);
        Array2DView<float4> dst2(mb.view().pointer(), camera_params_.depth.resolution);
        copy(pipeline_->debug_points_world_.readView(), dst2);
      }

      glPointSize(1.0f);

      GLSeparableProgram& vs = gl_state_->programs_["positionOnlyVS"];
      GLSeparableProgram& fs = gl_state_->programs_["drawSingleColorFS"];
      vs.setUniformMatrix4f(0, free_camera_.viewProjectionMatrix());
      fs.setUniformVector4f(0, { 1, 1, 1, 1 });
      gl_state_->draw_single_color_.bind();

      gl_state_->raycasted_points_.draw();

      GLProgramPipeline::unbindAll();
    }

    // Draw the color image on the 3D camera.
    if (false)
    {
        // TODO(jiawen): bindless texture
        const int kColorTextureUnit = 0;
        gl_state_->color_tracking_vis_texture_.bind(kColorTextureUnit);

        GLSeparableProgram& vs = gl_state_->programs_["drawTextureVS"];
        GLSeparableProgram& fs = gl_state_->programs_["drawTextureFS"];
        vs.setUniformMatrix4f(0, free_camera_.viewProjectionMatrix());
        fs.setUniformInt(0, kColorTextureUnit);
        gl_state_->draw_texture_.bind();

        gl_state_->tracked_rgb_camera_image_rect_.draw();

        gl_state_->color_tracking_vis_texture_.unbind(kColorTextureUnit);
        GLProgramPipeline::unbindAll();
    }

    // Draw the depth image on the 3D camera.
    if (false)
    {
      // TODO(jiawen): bindless texture
      const int kColorTextureUnit = 0;
      const int kColorTextureLocation = 0;
      gl_state_->depth_vis_texture_.bind(kColorTextureUnit);

      GLSeparableProgram& vs = gl_state_->programs_["drawTextureVS"];
      GLSeparableProgram& fs = gl_state_->programs_["drawTextureFS"];
      vs.setUniformMatrix4f(0, free_camera_.viewProjectionMatrix());
      fs.setUniformInt(kColorTextureLocation, kColorTextureUnit);
      gl_state_->draw_texture_.bind();

      gl_state_->tracked_depth_camera_image_rect_.draw();

      gl_state_->depth_vis_texture_.unbind(kColorTextureUnit);
      GLProgramPipeline::unbindAll();
    }

    // TODO(jiawen): only if kf has new stuff...
    {
      if (pipeline_->debug_smoothed_depth_vis_.notNull()) {
        gl_state_->smoothed_depth_vis_texture_.set(
          pipeline_->debug_smoothed_depth_vis_);
      }

      if (pipeline_->debug_incoming_camera_normals_.notNull()) {
        gl_state_->normal_vis_texture_.set(
          pipeline_->debug_incoming_camera_normals_);
      }

      if (pipeline_->debug_normals_world_.notNull()) {
        Array2DView<Vector4f> view(
          pipeline_->debug_normals_world_.pointer(),
          pipeline_->debug_normals_world_.size(),
          pipeline_->debug_normals_world_.stride());
        gl_state_->raycasted_normals_tex_.set(view);
      }
    }

    {
      glDisable(GL_DEPTH_TEST);

      Rect2f current_rect{ { 0, 0 }, 0.5f * gl_state_->color_texture_.size() };
      gl_state_->color_texture_.drawNV(current_rect);
      current_rect = translate(current_rect, { current_rect.width(), 0 });

      current_rect.size = 0.5f * gl_state_->color_tracking_vis_texture_.size();
      gl_state_->color_tracking_vis_texture_.drawNV(current_rect);
      current_rect = translate(current_rect, { current_rect.width(), 0 });

      current_rect.size = 0.5f * gl_state_->depth_vis_texture_.size();
      gl_state_->depth_vis_texture_.drawNV(current_rect);
      current_rect = translate(current_rect, { current_rect.width(), 0 });

      current_rect.size = 0.5f * gl_state_->smoothed_depth_vis_texture_.size();
      gl_state_->smoothed_depth_vis_texture_.drawNV(current_rect);
      current_rect = translate(current_rect, { current_rect.width(), 0 });

      current_rect.size = 0.5f * gl_state_->normal_vis_texture_.size();
      gl_state_->normal_vis_texture_.drawNV(current_rect);
      current_rect = translate(current_rect, { current_rect.width(), 0 });

      current_rect.size = 0.5f * gl_state_->compatibility_map_texture_.size();
      gl_state_->compatibility_map_texture_.drawNV(current_rect);

      current_rect = Rect2f{
        { 0, 0.5f * gl_state_->color_texture_.height() },
        gl_state_->raycasted_normals_tex_.size() };
      gl_state_->raycasted_normals_tex_.drawNV(current_rect);
      current_rect = translate(current_rect, { current_rect.width(), 0 });

      glEnable(GL_DEPTH_TEST);
    }
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

void MainWidget::UpdateCameraFrusta() {
  gl_state_->tracked_rgb_camera_.updatePositions(
    pipeline_->ColorCamera());

  // Frustum corners come in ccw order, flip to triangle strip order.
  std::vector<Vector3f> rgb_frustum_corners =
    pipeline_->ColorCamera().frustumCorners();
  gl_state_->tracked_rgb_camera_image_rect_.updatePositions({
    rgb_frustum_corners[0],
    rgb_frustum_corners[1],
    rgb_frustum_corners[3],
    rgb_frustum_corners[2]
  });

  gl_state_->tracked_depth_camera_.updatePositions(
    pipeline_->DepthCamera());

  // Frustum corners come in ccw order, flip to triangle strip order.
  std::vector<Vector3f> depth_frustum_corners =
    pipeline_->DepthCamera().frustumCorners();
  gl_state_->tracked_depth_camera_image_rect_.updatePositions({
    depth_frustum_corners[0],
    depth_frustum_corners[1],
    depth_frustum_corners[3],
    depth_frustum_corners[2]
  });
}

void MainWidget::DrawWorldAxes() {
  GLSeparableProgram& vs = gl_state_->programs_["drawColorVS"];
  vs.setUniformMatrix4f(0, free_camera_.viewProjectionMatrix());
  gl_state_->draw_color_.bind();

  gl_state_->world_axes_.draw();

  GLProgramPipeline::unbindAll();
}

void MainWidget::DrawCameraFrustaAndTSDFGrid() {
  GLSeparableProgram& vs = gl_state_->programs_["drawColorVS"];
  vs.setUniformMatrix4f(0, free_camera_.viewProjectionMatrix());
  gl_state_->draw_color_.bind();

  gl_state_->tracked_rgb_camera_.draw();
  gl_state_->tracked_depth_camera_.draw();
  gl_state_->tsdf_bbox_.draw();

  GLProgramPipeline::unbindAll();
}

void MainWidget::DrawBoardTexture() {
  // TODO(jiawen): bindless texture
  const int kColorTextureUnit = 0;
  gl_state_->board_texture_.bind(kColorTextureUnit);

  GLSeparableProgram& vs = gl_state_->programs_["drawTextureVS"];
  GLSeparableProgram& fs = gl_state_->programs_["drawTextureFS"];
  vs.setUniformMatrix4f(0, free_camera_.viewProjectionMatrix());
  fs.setUniformInt(0, kColorTextureUnit);
  gl_state_->draw_texture_.bind();

  gl_state_->board_rectangle_.draw();

  gl_state_->board_texture_.unbind(kColorTextureUnit);
  GLProgramPipeline::unbindAll();
}

void MainWidget::DrawUnprojectedPointCloud() {
  glPointSize(3.0f);

  const int kFreeCameraFromWorldLocation = 0;
  const int kDepthCameraFLPPLocation = 1;
  const int kDepthCameraRangeMinMaxLocation = 2;
  const int kDepthWorldFromCameraLocation = 3;
  const int kColorClipFromWorldLocation = 4;
  const int kDepthTextureLocation = 5;
  const int kColorTextureLocation = 6;

  const int kDepthTextureUnit = 0;
  const int kColorTextureUnit = 1;

  GLSeparableProgram& vs = gl_state_->programs_["unprojectPointCloudVS"];
  vs.setUniformMatrix4f(kFreeCameraFromWorldLocation,
    free_camera_.viewProjectionMatrix());
  vs.setUniformVector4f(kDepthCameraFLPPLocation,
  {
    camera_params_.depth.intrinsics.focalLength,
    camera_params_.depth.intrinsics.principalPoint
  }
  );
  vs.setUniformVector2f(kDepthCameraRangeMinMaxLocation,
    camera_params_.depth.depth_range.leftRight());
  vs.setUniformMatrix4f(kDepthWorldFromCameraLocation,
    pipeline_->DepthCamera().worldFromCamera().asMatrix());
  vs.setUniformMatrix4f(kColorClipFromWorldLocation,
    pipeline_->ColorCamera().viewProjectionMatrix());

  gl_state_->depth_texture_.bind(kDepthTextureUnit);
  gl_state_->nearest_sampler_.bind(kDepthTextureUnit);
  gl_state_->color_texture_.bind(kColorTextureUnit);
  gl_state_->linear_sampler_.bind(kColorTextureUnit);
  vs.setUniformInt(kDepthTextureLocation, kDepthTextureUnit);
  vs.setUniformInt(kColorTextureLocation, kColorTextureUnit);

  gl_state_->unproject_point_cloud_.bind();

  gl_state_->xy_coords_.draw();

  gl_state_->depth_texture_.unbind(kDepthTextureUnit);
  GLSamplerObject::unbind(kDepthTextureUnit);
  gl_state_->color_texture_.unbind(kColorTextureUnit);
  GLSamplerObject::unbind(kColorTextureUnit);
}
