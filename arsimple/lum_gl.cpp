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

#include "lum_gl.h"
#include <cstdio>

namespace lum {

uint32_t lglCreateBuffer() {
	uint32_t ret;
	glCreateBuffers(1, &ret);
	return ret;
}
uint32_t lglCreateVertexArray() {
	uint32_t ret;
	glCreateVertexArrays(1, &ret);
	return ret;
}
void lglDeleteBuffer(uint32_t buffer){
	glDeleteBuffers(1, &buffer);
}
void lglDeleteVertexArrays(uint32_t vao){
	glDeleteVertexArrays(1, &vao);
}
void lglBufferStorage(uint32_t b, size_t sz, void* data, GLenum flags) {
	glNamedBufferStorage(b, sz, data, flags);
}
void* lglMapBufferRW(uint32_t b) {
	return glMapNamedBuffer(b, GL_READ_WRITE);
}
void* lglMapBufferR(uint32_t b) {
	return glMapNamedBuffer(b, GL_READ_ONLY);
}
void* lglMapBufferW(uint32_t b) {
	return glMapNamedBuffer(b, GL_WRITE_ONLY);
}
void* lglMapBufferRangeRW(uint32_t b, int offset, int len) {
	return glMapNamedBufferRange(b, offset, len, GL_MAP_READ_BIT | GL_MAP_WRITE_BIT);
}
void* lglMapBufferRangeR(uint32_t b, int offset, int len) {
	return glMapNamedBufferRange(b, offset, len, GL_MAP_READ_BIT);
}
void* lglMapBufferRangeW(uint32_t b, int offset, int len) {
	return glMapNamedBufferRange(b, offset, len, GL_MAP_WRITE_BIT);
}
void lglUnmapBuffer(uint32_t buffer) {
	glUnmapNamedBuffer(buffer);
}
uint32_t lglCreateTexture(GLenum target) {
	uint32_t ret;
	glCreateTextures(target, 1, &ret);
	return ret;
}
void lglDeleteTexture(uint32_t texture) {
	glDeleteTextures(1, &texture);
}
uint32_t lglCreateFramebuffer() {
	uint32_t ret;
	glCreateFramebuffers(1, &ret);
	return ret;
}
void lglDeleteFramebuffer(uint32_t fb) {
	glDeleteFramebuffers(1, &fb);
}

zglFramebuffer zglCreateFramebuffer(int w, int h, GLenum colfmt, GLenum depthfmt) {
	zglFramebuffer fb;
	fb.fb = lglCreateFramebuffer();
	fb.texture = lglCreateTexture(GL_TEXTURE_2D);
	fb.depthtexture = lglCreateTexture(GL_TEXTURE_2D);
	fb.w = w;
	fb.h = h;
	// TODO figure out if mipmap for backbuffer makes a difference
	glTextureStorage2D(fb.texture, 1, colfmt, w, h);
	glTextureStorage2D(fb.depthtexture, 1, depthfmt, w, h);

	glTextureParameteri(fb.texture, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTextureParameteri(fb.texture, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	glNamedFramebufferTexture(fb.fb, GL_COLOR_ATTACHMENT0, fb.texture, 0);
	glNamedFramebufferTexture(fb.fb, GL_DEPTH_ATTACHMENT, fb.depthtexture, 0);
	glNamedFramebufferDrawBuffer(fb.fb, GL_COLOR_ATTACHMENT0);
	glNamedFramebufferReadBuffer(fb.fb, GL_COLOR_ATTACHMENT0);

	GLenum status = glCheckNamedFramebufferStatus(fb.fb, GL_DRAW_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE) {
		printf("Framebuffer incomplete: %x\n", status);
	}
	return fb;
}
zglFramebuffer zglCreateFramebufferMSAA(int w, int h, GLenum colfmt, GLenum depthfmt, int nsamples) {
	if (GLenum err = glGetError()) {
		printf("Error before fb: %d\n", err);
	}
	zglFramebuffer fb;
	fb.fb = lglCreateFramebuffer();
	fb.texture = lglCreateTexture(GL_TEXTURE_2D_MULTISAMPLE);
	fb.depthtexture = lglCreateTexture(GL_TEXTURE_2D_MULTISAMPLE);
	fb.w = w;
	fb.h = h;
	// TODO figure out if mipmap for backbuffer makes a difference
	glTextureStorage2DMultisample(fb.texture, nsamples, colfmt, w, h, GL_FALSE);
	glTextureStorage2DMultisample(fb.depthtexture, nsamples, depthfmt, w, h, GL_FALSE);

	glNamedFramebufferTexture(fb.fb, GL_COLOR_ATTACHMENT0, fb.texture, 0);
	glNamedFramebufferTexture(fb.fb, GL_DEPTH_ATTACHMENT, fb.depthtexture, 0);
	glNamedFramebufferDrawBuffer(fb.fb, GL_COLOR_ATTACHMENT0);
	glNamedFramebufferReadBuffer(fb.fb, GL_COLOR_ATTACHMENT0);

	GLenum status = glCheckNamedFramebufferStatus(fb.fb, GL_DRAW_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE) {
		printf("Framebuffer incomplete: %x\n", status);
	}
	if (GLenum err = glGetError()) {
		printf("Error after fb: %d\n", err);
	}
	return fb;
}
void zglDeleteFramebuffer(zglFramebuffer fb) {
	lglDeleteTexture(fb.texture);
	lglDeleteTexture(fb.depthtexture);
	lglDeleteFramebuffer(fb.fb);
}
void zglBindFramebuffer(zglFramebuffer fb) {
	glBindFramebuffer(GL_FRAMEBUFFER, fb.fb);
}
void zglUnbindFramebuffer() {
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
void zglClearFramebuffer(zglFramebuffer fb, const uint8_t* color) {
	glViewport(0, 0, fb.w, fb.h);
	glClearColor(color[0] / 255.0f, color[1] / 255.0f, color[2] / 255.0f, color[3] / 255.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
}