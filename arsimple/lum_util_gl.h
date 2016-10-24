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

//
// Error checking for OpenGL. Most convenient is to use a debug context
// and use KHR_debug_output through lum::setupDebugPrint()
//

#ifndef LUM_UTIL_GL_H
#define LUM_UTIL_GL_H

#include <iostream>
#include "lum_gl.h"
#include <vector>

namespace lum {

/* Error Checking */
void check_gl_err(GLenum err, int lineno, char* srcstr);
#define GL_CHECK  if(GLenum err_internal__ = glGetError()) {lum::check_gl_err(err_internal__, __LINE__, "");}
#define GL_CHECKED(A) A; if(GLenum err_internal__ = glGetError()) {lum::check_gl_err(err_internal__, __LINE__, #A);}

/**
 * Return formatted string with some information about the GL environment
 *
 * contains
 * * GL_MAJOR_VERSION
 * * GL_MINOR_VERSION
 * * GL_RENDERER
 * * GL_VENDOR
 * * GL_VERSION
 * * GL_SHADING_LANGUAGE_VERSION
 */
std::string PrintOpenGLInfo();

/** Returns true if we run GL version 3.2 or 3.3 */
bool gl32CoreProfile();
void setupDebugPrint(FILE*);

struct texinfo {
	int max_texture_size;
	int max_3d_texture_size;
	int max_array_texture_layers;

	int max_sparse_texture_size;
	int max_sparse_3d_texture_size;
	int max_sparse_array_texture_layers;

	int max_rectangle_texture_size;

	int max_compute_texture_image_units;
	int max_combined_texture_image_units;
	int max_geometry_texture_image_units;
	int max_texture_image_units;
	int max_vertex_texture_image_units;

	int min_program_texel_offset;
	int max_program_texel_offset;
};
struct computeinfo {
	int max_compute_shader_storage_blocks;
	int max_combined_shader_storage_blocks;
	int max_compute_uniform_blocks;
	int max_compute_texture_image_units;
	int max_compute_uniform_components;

	int max_compute_atomic_counters;
	int max_compute_atomic_counter_buffers;
	int max_combined_compute_uniform_components;
	int max_compute_work_group_invocations;
	int max_compute_work_group_count[3];
	int max_compute_work_group_size[3];
};
struct memstate_nvx {
	int dedicated_vidmem;
	int total_available_memory;
	int current_available_vidmem;
	int eviction_count;
	int evicted_memory;
};
struct shaderstorageinfo {
	int max_vertex_shader_storage_blocks;
	int max_geometry_shader_storage_blocks;
	int max_tess_control_shader_storage_blocks;
	int max_tess_evaluation_shader_storage_blocks;
	int max_fragment_shader_storage_blocks;
	int max_compute_shader_storage_blocks;
	int max_combined_shader_storage_blocks;
	int max_shader_storage_buffer_bindings;
	int max_shader_storage_block_size;
	int shader_storage_buffer_offset_alignment;
};

struct uniformbufferinfo {
	int max_vertex_uniform_blocks;
	int max_geometry_uniform_blocks;
	int max_fragment_uniform_blocks;
	int max_combined_uniform_blocks;
	int max_uniform_buffer_bindings;
	int max_uniform_block_size;
	int max_combined_vertex_uniform_components;
	int max_combined_geometry_uniform_components;
	int max_combined_fragment_uniform_components;
	int uniform_buffer_offset_alignment;
};
// PACK:   GPU -> RAM
// UNPACK: RAM -> GPU
struct packstate {
	bool swap_bytes;
	bool lsb_first;
	int row_length;
	int image_height;
	int skip_rows;
	int skip_pixels;
	int skip_images;
	int alignment;
};

texinfo getTextureInfo();
computeinfo getComputeInfo();
memstate_nvx getMemStateNVX();
packstate getPackState();
packstate getUnpackState();
shaderstorageinfo getShaderStorageInfo();
uniformbufferinfo getUniformBufferInfo();
}

#endif /* defined(__Relight__lum_util_gl__) */
