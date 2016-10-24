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

#ifndef LUM_TRANSFORM_H
#define LUM_TRANSFORM_H

#include "lum_linalg.h"
namespace lum {

	float deg2rad(float deg);
	float rad2deg(float rad);
	float radmod2pi(float p);

    mat4f rotateX(float angle);
    mat4f rotateY(float angle);
    mat4f rotateZ(float angle);

    mat4f scaleX(float sx);
    mat4f scaleY(float sy);
    mat4f scaleZ(float sz);

    mat4f scaleXYZ(float sx, float sy, float sz);
    mat4f scaleXYZ(float s);

    mat4f translateX(float x);
    mat4f translateY(float y);
    mat4f translateZ(float z);
    mat4f translateXYZ(float x, float y, float z);

    mat4f rotateTo(float x, float y, float z);

	// returns a matrix A s.t. A*e_x is in the plane, a*e_y is in the plane, and a*e_z is along the normal.
	mat4f planeBasis(const vec4f & plane);

#pragma region intrinsic
	// some libs express field-of-view as horizontal (x direction)
	// others as vertical (y direction). Conversion routine.

	float fovyTofovx(float fovy, float aspect);
	float fovxTofovy(float fovx, float aspect);
	/** Returns 4x4 projection matrix. See implementation comments for reference. */
	mat4f projectionMatrix(float fovy,
		float aspect,
		float nearZ,
		float farZ);

	mat4f orthoMatrix(float width, float height,
		float near, float far);
#pragma endregion intrinsic

    // transforms a 'unit y-up' object to span <from-to>
    mat4f fromTo(const vec3f & from, const vec3f & to);
	mat3f angleAxis(const vec3f& from, float alpha);
	mat3f crossProduct(const vec3f& c);
	mat3f rodrigues(const vec3f axis, float angle);
	void angleAxisToEuler(float x, float y, float z, float angle, float* yaw, float* pitch, float* roll);

    mat4f lookAt(const vec3f& eye, const vec3f& center, const vec3f& up);
}

#endif 
