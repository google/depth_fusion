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

#pragma once
#include "lum_unitquad.h"
#include <cstdint>

namespace lum {
class quadrenderer {
public:
	quadrenderer();
	quadrenderer(uint32_t texhandle);
	~quadrenderer();
	void draw();
	void draw(double now);
	void draw_with_tex(uint32_t texhandle);
	void draw_with_texlayer(uint32_t texhandle, int layer, int internalfmt);
	// data must hold w*h*3 bytes.
	void upload_texture(const unsigned char* data, int w, int h, int format, int type);
	float gamma;
	float black;
	float white;
	bool flipy;
	bool flipx;
	float rmask;
	float gmask;
	float bmask;
	float amask;

	void raonly();
	void gaonly();
	void baonly();
	void resetmasks();

	unitquad m_quad;
	unsigned int m_tex;
	unsigned int m_program;

	bool owns_texture;
private:
	void draw_inner();

};

class cubemaprenderer {
public:
	cubemaprenderer();
	~cubemaprenderer();
	void draw_with_tex(uint32_t texhandle);
	void draw_with_tex(uint32_t* views);
	// data must hold w*h*3 bytes.
	float gamma;
	float black;
	float white;
	bool flipy;

	unitquad m_quad;
	unsigned int m_program;
};

class sequencerenderer {
public:
	sequencerenderer(int nframes, int w, int h);
	~sequencerenderer();

	void upload_texture(const unsigned char* data, int nframe, int format, int type);
	void set_progress(float p);
	void draw();

	unitquad m_quad;
	int m_w;
	int m_h;
	float m_progress;
	unsigned int m_tex;
	unsigned int m_program;
};

class flowblender{
private:
	void upload_frame(unsigned int texid, const uint8_t* data, int format, int type);
public:
	flowblender(int w, int h);
	~flowblender();

	void upload_frame1(const unsigned char* data, int format, int type);
	void upload_frame2(const unsigned char* data, int format, int type);
	void upload_flow(const float* data);
	void set_progress(float p);
	void draw();

	unitquad m_quad;
	int m_w;
	int m_h;
	float m_progress;
	unsigned int m_tex1;
	unsigned int m_tex2;
	unsigned int m_flowtex;
	unsigned int m_program;

};

class flowsequence{
public:
	flowsequence(int w, int h, int nframes);
	~flowsequence();

	void upload_frame(int frameidx, const unsigned char* data, int format, int type);
	void upload_flow(int frameidx, const float* data);
	void upload_frames(const unsigned char* data, int format, int type);
	void upload_flow(const float* data);
	void set_progress(float p);
	void draw();

	unitquad m_quad;
	int m_nframes;
	int m_w;
	int m_h;
	float m_progress;
	unsigned int m_tex;
	unsigned int m_flowtex;
	unsigned int m_program;
};

class checkerquad {
public:
	checkerquad();
	~checkerquad();
	void draw();

	void next();

	int nstepsx;
	int nstepsy;
	int boxw;
	int boxh;
	int patternidx;

	unitquad m_quad;
	unsigned int m_tex;
	unsigned int m_program;
};

struct trirender_vertex{
	uint32_t color_bgra;
	float pos[3];
};
class trirenderer {
public:
	void init(int nverts, int nindices);
	void reset();
	void upload_vertex();
	void upload_index();
	void draw();

	int m_nverts;
	int m_nindices;

	trirender_vertex* m_vdata;
	uint16_t* m_idata;

	uint32_t m_program;
	uint32_t m_vao;
	uint32_t m_vbuf;
	uint32_t m_ibuf;
};
}