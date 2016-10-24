/*Copyright 2016 Google Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.*/


#include "lum_quadrenderer.h"
#include "lum_gl.h"
#include "lum_glprogram2.h"
#include <cassert>
#include <cstdio>
#include <cstdint>

namespace lum {
	static const char c_textured_vshader[] = "\
											 #version 150\n\
											 \n\
											 in vec3 VertexPosition;\
											 in vec2 VertexTexCoord;\
											 \
											 out vec2 v_vertexTexCoords;\
											 \
											 void main(){\
											 gl_Position = vec4(VertexPosition, 1); \
											 v_vertexTexCoords = VertexTexCoord; \
											 }";

	static const char c_textured_fshader[] = "\
											 #version 150\n\
											 in vec2 v_vertexTexCoords;\
											 out vec4 outputColor;\
											 uniform sampler2D tex;\
											 uniform float gamma;\
											 uniform float black;\
											 uniform float white;\
											 uniform bool flipy;\
											 uniform bool flipx;\
											 uniform vec4 colmask;\
											 \
											 void main() {\
											 vec2 uv = v_vertexTexCoords;\
											 if (flipy) {uv.y = 1 - uv.y;}\
											 if (flipx) {uv.x = 1 - uv.x;}\
											 vec4 texColor = texture(tex, uv);\
											 float alpha = texColor.w;\
											 texColor = texColor - vec4(black, black, black, 1); \
											 float scalef = 1.0 / (white - black); \
											 texColor = texColor * vec4(scalef, scalef, scalef, 1); \
											 texColor = pow(texColor, vec4(gamma)); \
											 texColor.w = alpha; \
											 outputColor = colmask * texColor;\
											 \
											 /*outputColor = texture(tex, v_vertexTexCoords);*/\
											 }";

	static const char c_sequence_fshader[] = "\
											 #version 150\n\
											 in vec2 v_vertexTexCoords;\
											 out vec4 outputColor;\
											 uniform sampler3D tex;\
											 uniform float progress;\
											 \
											 void main() {\
											 vec4 texColor = texture3D(tex, vec3(v_vertexTexCoords, progress));\
											 outputColor = texColor;\
											 }";


	static const char c_flowblender_fshader[] = "\
												#version 150\n\
												in vec2 v_vertexTexCoords;\
												out vec4 outputColor;\
												uniform sampler2D tex1;\
												uniform sampler2D tex2;\
												uniform sampler2D flowtex;\
												uniform float progress;\
												\
												void main() {\
												vec4 flow = texture(flowtex, v_vertexTexCoords);\
												vec2 flow1to2 = flow.xy;\
												vec2 flow2to1 = flow.zw;\
												ivec2 sz = textureSize(tex1, 0);\
												flow1to2 /= sz;\
												flow2to1 /= sz;\
												vec2 fetch1to2 = v_vertexTexCoords + progress * flow1to2;\
												vec2 fetch2to1 = v_vertexTexCoords + (1 - progress) * flow2to1;\
												vec4 c1 = texture(tex1, fetch1to2);\
												vec4 c2 = texture(tex2, fetch2to1);\
												\
												if (any(greaterThan(fetch1to2, vec2(1,1))) || any(lessThan(fetch1to2, vec2(0,0)))) {\
												outputColor = c2;\
												} else if (any(greaterThan(fetch2to1, vec2(1,1))) || any(lessThan(fetch2to1, vec2(0,0)))) {\
												outputColor = c1;\
												} else {\
												outputColor = (1 - progress) * c1 + progress * c2;\
												}\
												}";


	static const char c_flowsequence_fshader[] = "\
												 #version 150\n\
												 in vec2 v_vertexTexCoords;\
												 out vec4 outputColor;\
												 uniform sampler3D tex;\
												 uniform sampler3D flowtex;\
												 uniform float progress;\
												 uniform float flowidx;\
												 uniform float frame1idx;\
												 uniform float frame2idx;\
												 \
												 void main() {\
												 ivec3 sz = textureSize(tex, 0);\
												 vec4 flow = texture(flowtex, vec3(v_vertexTexCoords, (flowidx + 0.5) / sz.z));\
												 vec2 flow1to2 = flow.xy / sz.xy;\
												 vec2 flow2to1 = flow.zw / sz.xy;\
												 float frame1pane = (frame1idx + 0.5) / sz.z;\
												 float frame2pane = (frame2idx + 0.5) / sz.z;\
												 vec3 fetch1to2 = vec3(v_vertexTexCoords + progress * flow1to2, frame1pane);\
												 vec3 fetch2to1 = vec3(v_vertexTexCoords + (1 - progress) * flow2to1, frame2pane);\
												 vec4 c1 = texture(tex, fetch1to2);\
												 vec4 c2 = texture(tex, fetch2to1);\
												 \
												 if (v_vertexTexCoords.x < 0.4) {\
												 outputColor = c2;\
												 } else if (v_vertexTexCoords.x > 0.6) {\
												 outputColor = c1;\
												 } else {\
												 outputColor = (1 - progress) * c1 + progress * c2;\
												 }\
												 }";

	static const char c_checkered_vshader[] = "\
											  #version 150\n\
											  \n\
											  in vec3 VertexPosition;\
											  out vec2 v_vertexTexCoords;\
											  \
											  void main(){\
											  gl_Position = vec4(VertexPosition, 1); \
											  }";

	static const char c_checkered_fshader[] = "#version 150\n\
											  out vec4 outputColor;\
											  uniform int boxw;\
											  uniform int boxh;\
											  uniform int xoffset;\
											  uniform int yoffset;\
											  \
											  void main() {\
											  int c = int(gl_FragCoord.x); \
											  int r = int(gl_FragCoord.y); \
											  int rowbit = ((r + yoffset) / boxh) % 2;\
											  int colbit = ((c + xoffset) / boxw) % 2;\
											  float val = rowbit == colbit ? 255 : 0;\
											  outputColor = vec4(val, val, val, 1);\
											  }";

	quadrenderer::quadrenderer()
		:gamma(1), black(0), white(1), flipy(false), flipx(false), owns_texture(true), rmask(1.0f), gmask(1.0f), bmask(1.0f), amask(1.0f) {
		glGenTextures(1, &m_tex);
		m_program = linkProgram(c_textured_vshader, c_textured_fshader);
	}

	quadrenderer::quadrenderer(uint32_t texhandle)
		: gamma(1), black(0), white(1), flipy(false), flipx(false), m_tex(texhandle), owns_texture(false), rmask(1.0f), gmask(1.0f), bmask(1.0f), amask(1.0f) {
		m_program = linkProgram(c_textured_vshader, c_textured_fshader);
	}

	cubemaprenderer::cubemaprenderer()
		:gamma(1), black(0), white(1), flipy(false) {
		m_program = linkProgram(c_textured_vshader, c_textured_fshader);
	}
	cubemaprenderer::~cubemaprenderer() {
		glDeleteProgram(m_program);
	}

	quadrenderer::~quadrenderer()
	{
		if (owns_texture) {
			glDeleteTextures(1, &m_tex);
		}
		glDeleteProgram(m_program);
	}

	void quadrenderer::draw_with_tex(uint32_t texhandle) {
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, texhandle);
		draw_inner();
		glBindTexture(GL_TEXTURE_2D, 0);
	}
	void quadrenderer::draw_with_texlayer(uint32_t texhandle, int layer, int internalfmt) {
		uint32_t view;
		glGenTextures(1, &view);

		glTextureView(view, GL_TEXTURE_2D,
			texhandle, internalfmt,
			0, 1, // level
			layer, 1); // layer


		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, view);
		draw_inner();
		glBindTexture(GL_TEXTURE_2D, 0);
		glDeleteTextures(1, &view);
	}
	void quadrenderer::draw_inner() {
		glEnable(GL_BLEND);
		glDisable(GL_DEPTH_TEST);

		glUseProgram(m_program);

		glBindVertexArray(m_quad.vao);
		glBindFragDataLocation(m_program, 0, "outputColor");
		glBindAttribLocation(m_program, c_position_attr, "VertexPosition");
		glBindAttribLocation(m_program, c_uv_attr, "VertexTexCoord");

		int sampler_loc = glGetUniformLocation(m_program, "tex");
		glUniform1i(sampler_loc, 0);

		int gamma_loc = glGetUniformLocation(m_program, "gamma");
		glUniform1f(gamma_loc, gamma);

		int black_loc = glGetUniformLocation(m_program, "black");
		glUniform1f(black_loc, black);

		int white_loc = glGetUniformLocation(m_program, "white");
		glUniform1f(white_loc, white);
		
		int flipy_loc = glGetUniformLocation(m_program, "flipy");
		glUniform1i(flipy_loc, flipy);

		int flipx_loc = glGetUniformLocation(m_program, "flipx");
		glUniform1i(flipx_loc, flipx);

		int colmask = glGetUniformLocation(m_program, "colmask");
		glUniform4f(colmask, rmask, gmask, bmask, amask);

		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		GLenum err = glGetError();
		if (err) {
			printf("Error after draw()\n");
			assert(false);
		}
	}
	void quadrenderer::draw(){
		draw_with_tex(m_tex);
	}
	void quadrenderer::draw(double now){
		draw_with_tex(m_tex);
	}
	void quadrenderer::raonly() {
		rmask = 1.0f; amask = 1.0f;
		gmask = 0.0f; bmask = 0.0f;
	}
	void quadrenderer::gaonly() {
		gmask = 1.0f; amask = 1.0f;
		rmask = 0.0f; bmask = 0.0f;
	}
	void quadrenderer::baonly() {
		bmask = 1.0f; amask = 1.0f;
		rmask = 0.0f; gmask = 0.0f;
	}
	void quadrenderer::resetmasks() {
		rmask = 1.0f; amask = 1.0f;
		gmask = 1.0f; bmask = 1.0f;
	}

	void quadrenderer::upload_texture(const unsigned char* data, int w, int h, int format, int type) {
		if (GLenum err = glGetError()) {
			printf("Error before upload_texture()\n");
			assert(false);
		}
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, m_tex);

		static const GLint NO_BORDER = 0;

		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, NO_BORDER, format, type, data);
		//glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, format, type, nullptr);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glBindTexture(GL_TEXTURE_2D, 0);

		if (GLenum err = glGetError()) {
			printf("Error after upload_texture()\n");
			assert(false);
		}
	}

	void cubemaprenderer::draw_with_tex(uint32_t* views) {
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_BLEND);
		glDisable(GL_DEPTH_TEST);

		glUseProgram(m_program);

		glBindVertexArray(m_quad.vao);
		glBindFragDataLocation(m_program, 0, "outputColor");
		glBindAttribLocation(m_program, c_position_attr, "VertexPosition");
		glBindAttribLocation(m_program, c_uv_attr, "VertexTexCoord");

		int sampler_loc = glGetUniformLocation(m_program, "tex");
		glUniform1i(sampler_loc, 0);

		int gamma_loc = glGetUniformLocation(m_program, "gamma");
		glUniform1f(gamma_loc, gamma);

		int black_loc = glGetUniformLocation(m_program, "black");
		glUniform1f(black_loc, black);

		int white_loc = glGetUniformLocation(m_program, "white");
		glUniform1f(white_loc, white);
		
		int flipy_loc = glGetUniformLocation(m_program, "flipy");
		glUniform1i(flipy_loc, flipy);

		int colmask = glGetUniformLocation(m_program, "colmask");
		glUniform4f(colmask, 1.0f, 1.0f, 1.0f, 1.0f);

		int vp[4];
		glGetIntegerv(GL_VIEWPORT, vp);
		int cubew = vp[2] / 3;

		glActiveTexture(GL_TEXTURE0);
		for (int i = 0; i < 6; ++i) {
			glViewport(vp[0] + i % 3 * cubew, vp[1] + (i / 3) * cubew, cubew, cubew);

			glBindTexture(GL_TEXTURE_2D, views[i]);

			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
			glBindTexture(GL_TEXTURE_2D, 0);
			GLenum err = glGetError();
			if (err) {
				printf("Error after draw()\n");
				assert(false);
			}
		}
		glViewport(vp[0], vp[1], vp[2], vp[3]);
	}
	void cubemaprenderer::draw_with_tex(uint32_t cubemap) {
		glActiveTexture(GL_TEXTURE0);
		uint32_t texviews[6];
		glGenTextures(6, texviews);

		for (int i = 0; i < 6; ++i) {
			glTextureView(texviews[i], GL_TEXTURE_2D, cubemap, GL_RGBA32F, 0, 1, i, 1);
		}
		this->draw_with_tex(texviews);
		glDeleteTextures(6, texviews);
	}

	sequencerenderer::sequencerenderer(int nframes, int w, int h)
		: m_w(w), m_h(h), m_progress(0.0f) {
		glGenTextures(1, &m_tex);
		glBindTexture(GL_TEXTURE_3D, m_tex);
		glTexStorage3D(GL_TEXTURE_3D, 1, GL_RGBA8, w, h, nframes);
		if (GLenum err = glGetError()) {
			printf("Error beforelink\n");
			assert(false);
		}

		m_program = linkProgram(c_textured_vshader, c_sequence_fshader);
		if (GLenum err = glGetError()) {
			printf("Error after link\n");
			assert(false);
		}
	}
	sequencerenderer::~sequencerenderer(){
		glDeleteTextures(1, &m_tex);
		glDeleteProgram(m_program);
	}

	void sequencerenderer::upload_texture(const unsigned char* data, int nframe, int format, int type) {
		if (GLenum err = glGetError()) {
			printf("Error before upload_texture()\n");
			assert(false);
		}
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_3D, m_tex);

		glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, nframe, m_w, m_h, 1, format, type, data);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glBindTexture(GL_TEXTURE_3D, 0);

		if (GLenum err = glGetError()) {
			printf("Error after upload_texture()\n");
			assert(false);
		}
	}

	void sequencerenderer::set_progress(float p) {
		m_progress = p;
	}
	void sequencerenderer::draw() {
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_DEPTH_TEST);

		glUseProgram(m_program);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_3D, m_tex);

		glBindVertexArray(m_quad.vao);
		glBindFragDataLocation(m_program, 0, "outputColor");
		glBindAttribLocation(m_program, c_position_attr, "VertexPosition");
		glBindAttribLocation(m_program, c_uv_attr, "VertexTexCoord");

		int sampler_loc = glGetUniformLocation(m_program, "tex");
		glUniform1i(sampler_loc, 0);

		int progress_loc = glGetUniformLocation(m_program, "progress");
		glUniform1f(progress_loc, m_progress);


		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		GLenum err = glGetError();
		if (err) {
			printf("Error after draw()\n");
			assert(false);
		}
	}

	flowblender::flowblender(int w, int h) : m_w(w), m_h(h){
		glGenTextures(1, &m_tex1);
		glGenTextures(1, &m_tex2);
		glGenTextures(1, &m_flowtex);
		glBindTexture(GL_TEXTURE_2D, m_tex1);
		glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, w, h);
		glBindTexture(GL_TEXTURE_2D, m_tex2);
		glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, w, h);
		glBindTexture(GL_TEXTURE_2D, m_flowtex);
		glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA32F, w, h);

		if (GLenum err = glGetError()) {
			printf("Error beforelink\n");
			assert(false);
		}

		m_program = linkProgram(c_textured_vshader, c_flowblender_fshader);
		if (GLenum err = glGetError()) {
			printf("Error after link\n");
			assert(false);
		}
	}
	flowblender::~flowblender() {
		glDeleteTextures(1, &m_tex1);
		glDeleteTextures(1, &m_tex2);
		glDeleteTextures(1, &m_flowtex);
		glDeleteProgram(m_program);
	}

	void flowblender::upload_frame(unsigned int texid, const uint8_t* data, int format, int type) {
		if (GLenum err = glGetError()) {
			printf("Error before upload_texture()\n");
			assert(false);
		}
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, texid);

		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_w, m_h, format, type, data);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glBindTexture(GL_TEXTURE_2D, 0);

		if (GLenum err = glGetError()) {
			printf("Error after upload_texture()\n");
			assert(false);
		}
	}
	void flowblender::upload_frame1(const unsigned char* data, int format, int type) {
		upload_frame(m_tex1, data, format, type);
	}
	void flowblender::upload_frame2(const unsigned char* data, int format, int type) {
		upload_frame(m_tex2, data, format, type);
	}
	void flowblender::upload_flow(const float* data) {
		upload_frame(m_flowtex, reinterpret_cast<const uint8_t*>(data), GL_RGBA, GL_FLOAT);
	}
	void flowblender::set_progress(float p) {
		m_progress = p;
	}
	void flowblender::draw() {
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_DEPTH_TEST);

		glUseProgram(m_program);

		glBindVertexArray(m_quad.vao);
		glBindFragDataLocation(m_program, 0, "outputColor");
		glBindAttribLocation(m_program, c_position_attr, "VertexPosition");
		glBindAttribLocation(m_program, c_uv_attr, "VertexTexCoord");

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, m_tex1);
		int sampler_loc = glGetUniformLocation(m_program, "tex1");
		glUniform1i(sampler_loc, 0);

		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, m_tex2);
		sampler_loc = glGetUniformLocation(m_program, "tex2");
		glUniform1i(sampler_loc, 1);

		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_2D, m_flowtex);
		sampler_loc = glGetUniformLocation(m_program, "flowtex");
		glUniform1i(sampler_loc, 2);

		int progress_loc = glGetUniformLocation(m_program, "progress");
		glUniform1f(progress_loc, m_progress);

		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		GLenum err = glGetError();
		if (err) {
			printf("Error after draw()\n");
			assert(false);
		}
	}


	flowsequence::flowsequence(int w, int h, int nframes) : m_w(w), m_h(h), m_nframes(nframes), m_progress(0.0) {
		glGenTextures(1, &m_tex);
		glGenTextures(1, &m_flowtex);
		glBindTexture(GL_TEXTURE_3D, m_tex);
		glTexStorage3D(GL_TEXTURE_3D, 1, GL_RGBA8, w, h, m_nframes);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		glBindTexture(GL_TEXTURE_3D, m_flowtex);
		glTexStorage3D(GL_TEXTURE_3D, 1, GL_RGBA32F, w, h, m_nframes);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		if (GLenum err = glGetError()) {
			printf("Error beforelink\n");
			assert(false);
		}

		m_program = linkProgram(c_textured_vshader, c_flowsequence_fshader);
		if (GLenum err = glGetError()) {
			printf("Error after link\n");
			assert(false);
		}
	}

	flowsequence::~flowsequence() {
		glDeleteTextures(1, &m_tex);
		glDeleteTextures(1, &m_flowtex);
		glDeleteProgram(m_program);
	}

	void flowsequence::upload_frame(int frameidx, const unsigned char* data, int format, int type) {
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_3D, m_tex);

		glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, frameidx, m_w, m_h, 1, format, type, data);
		glBindTexture(GL_TEXTURE_3D, 0);
	}
	void flowsequence::upload_flow(int frameidx, const float* data) {
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_3D, m_flowtex);

		glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, frameidx, m_w, m_h, 1, GL_RGBA, GL_FLOAT, data);
		glBindTexture(GL_TEXTURE_3D, 0);
	}
	void flowsequence::upload_frames(const unsigned char* data, int format, int type) {
		if (GLenum err = glGetError()) {
			printf("Error before upload_texture()\n");
			assert(false);
		}
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_3D, m_tex);

		glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, 0, m_w, m_h, m_nframes, format, type, data);
		glBindTexture(GL_TEXTURE_3D, 0);

		if (GLenum err = glGetError()) {
			printf("Error after upload_texture()\n");
			assert(false);
		}
	}

	void flowsequence::upload_flow(const float* data) {
		if (GLenum err = glGetError()) {
			printf("Error before upload_texture()\n");
			assert(false);
		}
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_3D, m_flowtex);

		glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, 0, m_w, m_h, m_nframes, GL_RGBA, GL_FLOAT, data);
		glBindTexture(GL_TEXTURE_3D, 0);

		if (GLenum err = glGetError()) {
			printf("Error after upload_texture()\n");
			assert(false);
		}
	}

	void flowsequence::set_progress(float p){
		m_progress = p;
	}

	void flowsequence::draw(){
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_DEPTH_TEST);

		glUseProgram(m_program);

		glBindVertexArray(m_quad.vao);
		glBindFragDataLocation(m_program, 0, "outputColor");
		glBindAttribLocation(m_program, c_position_attr, "VertexPosition");
		glBindAttribLocation(m_program, c_uv_attr, "VertexTexCoord");

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_3D, m_tex);
		int sampler_loc = glGetUniformLocation(m_program, "tex");
		glUniform1i(sampler_loc, 0);

		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_3D, m_flowtex);
		sampler_loc = glGetUniformLocation(m_program, "flowtex");
		glUniform1i(sampler_loc, 1);

		int progress_loc = glGetUniformLocation(m_program, "progress");
		float progress_global = m_progress * m_nframes;
		float intpart;
		float within_frame_progess = modf(progress_global, &intpart);
		glUniform1f(progress_loc, within_frame_progess);


		int frame1idx = (int)floor(m_progress * m_nframes) % m_nframes;

		int loc = glGetUniformLocation(m_program, "flowidx");
		glUniform1f(loc, (float)frame1idx);


		int frame2idx = (frame1idx + 1) % m_nframes;
		//printf("Within Frame prog %.3f\nFrame 1 idx %d\nFrame 2 idx %d\n", within_frame_progess, frame1idx, frame2idx);
		loc = glGetUniformLocation(m_program, "frame1idx");
		glUniform1f(loc, (float)frame1idx);
		loc = glGetUniformLocation(m_program, "frame2idx");
		glUniform1f(loc, (float)frame2idx);


		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		GLenum err = glGetError();
		if (err) {
			printf("Error after draw()\n");
			assert(false);
		}
	}

	void  checkerquad::next() {
		patternidx++;
	}

	checkerquad::checkerquad()
		:nstepsx(3), nstepsy(3), boxw(20), boxh(20), patternidx(0) {
		m_program = linkProgram(c_checkered_vshader, c_checkered_fshader);
	}
	checkerquad::~checkerquad() {}
	void checkerquad::draw(){
		glDisable(GL_BLEND);
		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_DEPTH_TEST);

		glUseProgram(m_program);
		glBindVertexArray(m_quad.vao);
		glBindFragDataLocation(m_program, 0, "outputColor");
		glBindAttribLocation(m_program, c_position_attr, "VertexPosition");

		int loc = glGetUniformLocation(m_program, "boxw");
		glUniform1i(loc, boxw);

		loc = glGetUniformLocation(m_program, "boxh");
		glUniform1i(loc, boxh);

		const int c_xperiod = 2 * boxw;
		const int c_yperiod = 2 * boxh;
		const float xincrement = (float)c_xperiod / (float)nstepsx;
		const float yincrement = (float)c_yperiod / (float)nstepsy;

		int xstep = patternidx % nstepsx;
		int ystep = (patternidx / nstepsx) % nstepsy;
		int xoffset = (int)(xstep * xincrement);
		int yoffset = (int)(ystep * yincrement);
		loc = glGetUniformLocation(m_program, "xoffset");
		glUniform1i(loc, xoffset);

		loc = glGetUniformLocation(m_program, "yoffset");
		glUniform1i(loc, yoffset);

		GLenum err = glGetError();
		if (err) {
			printf("Error before draw()\n");
			assert(false);
		}
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		err = glGetError();
		if (err) {
			printf("Error after draw()\n");
			assert(false);
		}
	}
	void trirenderer::init(int nverts, int nindices) {
		m_nverts = nverts;
		m_nindices = nindices;

		m_vdata = new trirender_vertex[nverts];
		m_idata = new uint16_t[nindices];

		std::string vshader = readfile("../shaders/trirenderer.vert");
		std::string fshader = readfile("../shaders/trirenderer.frag");
		m_program = linkProgram(vshader.c_str(), fshader.c_str());
		glCreateBuffers(1, &m_ibuf);
		glCreateBuffers(1, &m_vbuf);
		// TODO experiment with persistent mapped buffer here.
		glNamedBufferStorage(m_ibuf, m_nindices * sizeof(uint16_t), nullptr, GL_MAP_WRITE_BIT | GL_DYNAMIC_STORAGE_BIT);
		glNamedBufferStorage(m_vbuf, m_nverts* sizeof(trirender_vertex), nullptr, GL_MAP_WRITE_BIT | GL_DYNAMIC_STORAGE_BIT);
		glCreateVertexArrays(1, &m_vao);

		glBindVertexArray(m_vao);
		glBindBuffer(GL_ARRAY_BUFFER, m_vbuf);

		// POSITION 3f
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(
			0,
			3,                  // number of elements per vertex, here (x,y,z)
			GL_FLOAT,           // the type of each element
			GL_FALSE,           // take our values as-is
			sizeof(trirender_vertex),                  // no extra data between each position
			(GLubyte*)(sizeof(uint32_t)) // offset of first element
			);

		// COLOR BGRA
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(
			1,
			4,                  // number of elements per vertex, here (x,y,z)
			GL_UNSIGNED_BYTE,           // the type of each element
			GL_TRUE,           // normalize
			sizeof(trirender_vertex),                  // no extra data between each position
			(GLubyte*)(0) // offset of first element
			);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibuf);
		glBindVertexArray(0);
	}
	void trirenderer::reset() {
		delete[] m_vdata;
		delete[] m_idata;

		glDeleteProgram(m_program);
		glDeleteBuffers(1, &m_ibuf);
		glDeleteBuffers(1, &m_vbuf);
		glDeleteVertexArrays(1, &m_vao);
	}
	void trirenderer::upload_vertex() {
		void* m = glMapNamedBuffer(m_vbuf, GL_WRITE_ONLY);
		memcpy(m, m_vdata, m_nverts * sizeof(trirender_vertex));
		glUnmapNamedBuffer(m_vbuf);
	}
	void trirenderer::upload_index() {
		void* m = glMapNamedBuffer(m_ibuf, GL_WRITE_ONLY);
		memcpy(m, m_idata, m_nindices * sizeof(uint16_t));
		glUnmapNamedBuffer(m_ibuf);
	}
	void trirenderer::draw() {
		glUseProgram(m_program);
		glBindVertexArray(m_vao);
		glDrawElements(GL_TRIANGLES, m_nindices, GL_UNSIGNED_SHORT, 0);
	}
}
