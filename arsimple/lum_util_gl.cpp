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

#include <vector>
#include <iterator>
#include <fstream>
#include <tuple>
#include <cassert>

#include "lum_util_gl.h"
#include "lum_gl.h"

namespace lum {

void check_gl_err(GLenum err, int lineno, char* srcstr) {
	printf("GL Error %x in Line %d: %s;\n", err, lineno, srcstr);
	assert(false);
}

bool gl32CoreProfile() {
	GLint major, minor;
	glGetIntegerv(GL_MAJOR_VERSION, &major);
	glGetIntegerv(GL_MINOR_VERSION, &minor);
	if (major > 3) {
		return true;
	}
	else {
		return  major == 3 && minor >= 2;
	}
}

std::string PrintOpenGLInfo()  {
	const GLubyte *renderer = glGetString(GL_RENDERER);
	const GLubyte *vendor = glGetString(GL_VENDOR);
	const GLubyte *version = glGetString(GL_VERSION);
	const GLubyte *glslVersion = glGetString(GL_SHADING_LANGUAGE_VERSION);
	GLint major, minor;
	glGetIntegerv(GL_MAJOR_VERSION, &major);
	glGetIntegerv(GL_MINOR_VERSION, &minor);

	std::string ret;
	const size_t buffsz = 1024;
	char buff[buffsz];
	_snprintf_s(buff, buffsz, "GL Vendor : %s\n", vendor);
	ret += buff;
	_snprintf_s(buff, buffsz, "GL Renderer : %s\n", renderer);
	ret += buff;
	_snprintf_s(buff, buffsz, "GL Version (string) : %s\n", version);
	ret += buff;
	_snprintf_s(buff, buffsz, "GL Version (integer) : %d.%d\n", major, minor);
	ret += buff;
	_snprintf_s(buff, buffsz, "GLSL Version : %s\n", glslVersion);
	ret += buff;
	return ret;
}

static FILE* debug_outputfile = stdout;
void GLAPIENTRY gl_dbg_callback(GLenum source,
	GLenum type,
	GLuint id,
	GLenum severity,
	GLsizei length,
	const GLchar* message,
	const void* userParam){
	// 131154 is pixel performance warning for glReadPixels
	if (severity > GL_DEBUG_SEVERITY_NOTIFICATION) {
		if (id != 131204 && id != 131076 && id != 131184 && id != 131186 && id != 131188 && id != 131154) {
			fprintf(debug_outputfile, "Type = %d, id = %d, severity = %d, %s\n", type, id, severity, message);
		}
	}
	return;
}

void setupDebugPrint(FILE* fh) {
	debug_outputfile = fh;
	glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
	glDebugMessageCallback(gl_dbg_callback, nullptr);
	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
}

texinfo getTextureInfo(){
	texinfo ret;
	/*
	Python:
	for s in S:
	print "glGetIntegerv(GL_%s, &ret.%s);" %(s.upper(), s)
	*/
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &ret.max_texture_size);
	glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE, &ret.max_3d_texture_size);
	glGetIntegerv(GL_MAX_ARRAY_TEXTURE_LAYERS, &ret.max_array_texture_layers);
	glGetIntegerv(GL_MAX_SPARSE_TEXTURE_SIZE_ARB, &ret.max_sparse_texture_size);
	glGetIntegerv(GL_MAX_SPARSE_3D_TEXTURE_SIZE_ARB, &ret.max_sparse_3d_texture_size);
	glGetIntegerv(GL_MAX_SPARSE_ARRAY_TEXTURE_LAYERS, &ret.max_sparse_array_texture_layers);
	glGetIntegerv(GL_MAX_RECTANGLE_TEXTURE_SIZE, &ret.max_rectangle_texture_size);
	glGetIntegerv(GL_MAX_COMPUTE_TEXTURE_IMAGE_UNITS, &ret.max_compute_texture_image_units);
	glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &ret.max_combined_texture_image_units);
	glGetIntegerv(GL_MAX_GEOMETRY_TEXTURE_IMAGE_UNITS, &ret.max_geometry_texture_image_units);
	glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &ret.max_texture_image_units);
	glGetIntegerv(GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS, &ret.max_vertex_texture_image_units);
	glGetIntegerv(GL_MIN_PROGRAM_TEXEL_OFFSET, &ret.min_program_texel_offset);
	glGetIntegerv(GL_MAX_PROGRAM_TEXEL_OFFSET, &ret.max_program_texel_offset);
	return ret;
}

computeinfo getComputeInfo() {
	computeinfo ret;
	glGetIntegerv(GL_MAX_COMPUTE_SHADER_STORAGE_BLOCKS, &ret.max_compute_shader_storage_blocks);
	glGetIntegerv(GL_MAX_COMBINED_SHADER_STORAGE_BLOCKS, &ret.max_combined_shader_storage_blocks);
	glGetIntegerv(GL_MAX_COMPUTE_UNIFORM_BLOCKS, &ret.max_compute_uniform_blocks);
	glGetIntegerv(GL_MAX_COMPUTE_TEXTURE_IMAGE_UNITS, &ret.max_compute_texture_image_units);
	glGetIntegerv(GL_MAX_COMPUTE_UNIFORM_COMPONENTS, &ret.max_compute_uniform_components);
	glGetIntegerv(GL_MAX_COMPUTE_ATOMIC_COUNTERS, &ret.max_compute_atomic_counters);
	glGetIntegerv(GL_MAX_COMPUTE_ATOMIC_COUNTER_BUFFERS, &ret.max_compute_atomic_counter_buffers);
	glGetIntegerv(GL_MAX_COMBINED_COMPUTE_UNIFORM_COMPONENTS, &ret.max_combined_compute_uniform_components);
	glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &ret.max_compute_work_group_invocations);
	for (int i = 0; i < 3; ++i){
		glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, i, &ret.max_compute_work_group_count[i]);
		glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, i, &ret.max_compute_work_group_size[i]);
	}
	return ret;
}

memstate_nvx getMemStateNVX() {
	memstate_nvx ret;
	glGetIntegerv(GL_GPU_MEMORY_INFO_DEDICATED_VIDMEM_NVX, &ret.dedicated_vidmem);
	glGetIntegerv(GL_GPU_MEMORY_INFO_TOTAL_AVAILABLE_MEMORY_NVX, &ret.total_available_memory);
	glGetIntegerv(GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX, &ret.current_available_vidmem);
	glGetIntegerv(GL_GPU_MEMORY_INFO_EVICTION_COUNT_NVX, &ret.eviction_count);
	glGetIntegerv(GL_GPU_MEMORY_INFO_EVICTED_MEMORY_NVX, &ret.evicted_memory);
	return ret;
}
packstate getPackState() {
	packstate ret;
	glGetIntegerv(GL_PACK_SWAP_BYTES, (int*)&ret.swap_bytes);
	glGetIntegerv(GL_PACK_LSB_FIRST, (int*)&ret.lsb_first);
	glGetIntegerv(GL_PACK_ROW_LENGTH, &ret.row_length);
	glGetIntegerv(GL_PACK_IMAGE_HEIGHT, &ret.image_height);
	glGetIntegerv(GL_PACK_SKIP_ROWS, &ret.skip_rows);
	glGetIntegerv(GL_PACK_SKIP_PIXELS, &ret.skip_pixels);
	glGetIntegerv(GL_PACK_SKIP_IMAGES, &ret.skip_images);
	glGetIntegerv(GL_PACK_ALIGNMENT, &ret.alignment);
	return ret;
}
packstate getUnpackState() {
	packstate ret;
	glGetIntegerv(GL_UNPACK_SWAP_BYTES, (int*)&ret.swap_bytes);
	glGetIntegerv(GL_UNPACK_LSB_FIRST, (int*)&ret.lsb_first);
	glGetIntegerv(GL_UNPACK_ROW_LENGTH, &ret.row_length);
	glGetIntegerv(GL_UNPACK_IMAGE_HEIGHT, &ret.image_height);
	glGetIntegerv(GL_UNPACK_SKIP_ROWS, &ret.skip_rows);
	glGetIntegerv(GL_UNPACK_SKIP_PIXELS, &ret.skip_pixels);
	glGetIntegerv(GL_UNPACK_SKIP_IMAGES, &ret.skip_images);
	glGetIntegerv(GL_UNPACK_ALIGNMENT, &ret.alignment);
	return ret;
}
shaderstorageinfo getShaderStorageInfo() {
	shaderstorageinfo ret;
	glGetIntegerv(GL_MAX_VERTEX_SHADER_STORAGE_BLOCKS, &ret.max_vertex_shader_storage_blocks);
	glGetIntegerv(GL_MAX_GEOMETRY_SHADER_STORAGE_BLOCKS, &ret.max_geometry_shader_storage_blocks);
	glGetIntegerv(GL_MAX_TESS_CONTROL_SHADER_STORAGE_BLOCKS, &ret.max_tess_control_shader_storage_blocks);
	glGetIntegerv(GL_MAX_TESS_EVALUATION_SHADER_STORAGE_BLOCKS, &ret.max_tess_evaluation_shader_storage_blocks);
	glGetIntegerv(GL_MAX_FRAGMENT_SHADER_STORAGE_BLOCKS, &ret.max_fragment_shader_storage_blocks);
	glGetIntegerv(GL_MAX_COMPUTE_SHADER_STORAGE_BLOCKS, &ret.max_compute_shader_storage_blocks);
	glGetIntegerv(GL_MAX_COMBINED_SHADER_STORAGE_BLOCKS, &ret.max_combined_shader_storage_blocks);
	glGetIntegerv(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS, &ret.max_shader_storage_buffer_bindings);
	glGetIntegerv(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &ret.max_shader_storage_block_size);
	glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &ret.shader_storage_buffer_offset_alignment);
	return ret;
}
uniformbufferinfo getUniformBufferInfo() {
	uniformbufferinfo ret;
	glGetIntegerv(GL_MAX_VERTEX_UNIFORM_BLOCKS, &ret.max_vertex_uniform_blocks);
	glGetIntegerv(GL_MAX_GEOMETRY_UNIFORM_BLOCKS, &ret.max_geometry_uniform_blocks);
	glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_BLOCKS, &ret.max_fragment_uniform_blocks);
	glGetIntegerv(GL_MAX_COMBINED_UNIFORM_BLOCKS, &ret.max_combined_uniform_blocks);
	glGetIntegerv(GL_MAX_UNIFORM_BUFFER_BINDINGS, &ret.max_uniform_buffer_bindings);
	glGetIntegerv(GL_MAX_UNIFORM_BLOCK_SIZE, &ret.max_uniform_block_size);
	glGetIntegerv(GL_MAX_COMBINED_VERTEX_UNIFORM_COMPONENTS, &ret.max_combined_vertex_uniform_components);
	glGetIntegerv(GL_MAX_COMBINED_GEOMETRY_UNIFORM_COMPONENTS, &ret.max_combined_geometry_uniform_components);
	glGetIntegerv(GL_MAX_COMBINED_FRAGMENT_UNIFORM_COMPONENTS, &ret.max_combined_fragment_uniform_components);
	glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &ret.uniform_buffer_offset_alignment);
	return ret;
}
}