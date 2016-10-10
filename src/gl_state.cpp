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
#include "gl_state.h"

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

GLState::GLState(
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
		GLImageInternalFormat::RGBA32F) {
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
	board_texture_.setSwizzleRGBA(swizzle_rrr1);
	depth_vis_texture_.setSwizzleRGBA(swizzle_rrr1);
	smoothed_depth_vis_texture_.setSwizzleRGBA(swizzle_rrr1);

	nearest_sampler_.setMinMagFilterModes(GLTextureFilterMode::NEAREST);
	nearest_sampler_.setWrapModes(GLWrapMode::CLAMP_TO_EDGE);

	linear_sampler_.setMinMagFilterModes(GLTextureFilterMode::LINEAR);
	linear_sampler_.setWrapModes(GLWrapMode::CLAMP_TO_EDGE);
}