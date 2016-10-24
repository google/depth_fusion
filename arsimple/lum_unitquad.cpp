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

#include "lum_unitquad.h"
#include "lum_gl.h"
#include <cassert>

namespace lum {

unitquad::unitquad()
{
	glGenVertexArrays(1, &vao);
	glGenBuffers(1, &m_buffer);

	// upload quad data.
	glBindVertexArray(vao);

	// attrs: (x y z u v)
	float attrs[] = { -1, -1, 0, 0, 1,  // bottom left
		1, -1, 0, 1, 1,                 // bottom right
		1, 1, 0, 1, 0,                  // top right
		-1, 1, 0, 0, 0 };			    // top left
	glBindBuffer(GL_ARRAY_BUFFER, m_buffer);
	glBufferData(GL_ARRAY_BUFFER, 4 * 5 *  sizeof(float), attrs, GL_STATIC_DRAW);

	glEnableVertexAttribArray(c_position_attr);
	glVertexAttribPointer( c_position_attr,
		3,                  // number of elements per vertex, here (x,y,z)
		GL_FLOAT,           // the type of each element
		GL_FALSE,           // take our values as-is
		5*sizeof(float),    // no extra data between each position
		(GLubyte*)0         // offset of first element
		);

	glEnableVertexAttribArray(c_uv_attr);
	glVertexAttribPointer( c_uv_attr,
		2,                  // number of elements per vertex, here (u, v)
		GL_FLOAT,           // the type of each element
		GL_FALSE,           // take our values as-is
		5*sizeof(float),                  // no extra data between each position
		(GLubyte*)(3*sizeof(float))      // offset of first element
		);

	glGenBuffers(1, &m_indices);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indices);

	unsigned int indices[] = { 0, 1, 2, 2, 3, 0 };
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(unsigned int), indices, GL_STATIC_DRAW);

	glBindVertexArray(0);
	assert(m_buffer);
	assert(m_indices);
	assert(vao);
}


unitquad::~unitquad()
{
	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(1, &m_buffer);
	glDeleteBuffers(1, &m_indices);
}

}
