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

#ifndef LUM_GL_H
#define LUM_GL_H

#include <cstdint>
#include <GL/glew.h>

#define PUSH_GROUP(ID, MSG)do {glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, ID, (GLsizei)strlen(MSG), MSG);} while(0);
#define POP_GROUP() do {glPopDebugGroup();} while(0);
#define CEIL(N, D) (((N) + (D) - 1) / (D))

#define GLSL_STRUCT struct // has matching GLSL struct

namespace lum {

/*
* simple helper functions that streamline
* frequently-used opengl interfaces.
* Do not introduce new types and do not manage lifetime.
*/
uint32_t lglCreateBuffer();
uint32_t lglCreateVertexArray();
void lglDeleteBuffer(uint32_t buffer);
void lglDeleteVertexArrays(uint32_t vao);
void lglBufferStorage(uint32_t b, size_t sz, void* data = nullptr,
	GLenum flags = GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT | GL_MAP_WRITE_BIT);
void* lglMapBufferRW(uint32_t b);
void* lglMapBufferR(uint32_t b);
void* lglMapBufferW(uint32_t b);
void* lglMapBufferRangeRW(uint32_t b, int offset, int len);
void* lglMapBufferRangeR(uint32_t b, int offset, int len);
void* lglMapBufferRangeW(uint32_t b, int offset, int len);
void lglUnmapBuffer(uint32_t buffer);
uint32_t lglCreateTexture(GLenum target);
void lglDeleteTexture(uint32_t texture);
uint32_t lglCreateFramebuffer();
void lglDeleteFramebuffer(uint32_t fb);


/*\
* zgl: higher level opengl helpers and compound objects
*/
struct zglFramebuffer {
	uint32_t fb;
	uint32_t texture;
	uint32_t depthtexture;
	int w;
	int h;
};
zglFramebuffer zglCreateFramebuffer(int w, int h, GLenum colfmt = GL_RGBA8, GLenum depthfmt = GL_DEPTH_COMPONENT32F);
zglFramebuffer zglCreateFramebufferMSAA(int w, int h, GLenum colfmt = GL_RGBA8, GLenum depthfmt = GL_DEPTH_COMPONENT32F, int nsamples = 4);
void zglDeleteFramebuffer(zglFramebuffer fb);
void zglBindFramebuffer(zglFramebuffer fb);
void zglUnbindFramebuffer();
void zglClearFramebuffer(zglFramebuffer fb, const uint8_t* color);

} // namespace lum

#endif
