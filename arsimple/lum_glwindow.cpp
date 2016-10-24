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

#include "lum_glwindow.h"

#include <cassert>
#include "lum_gl.h"
#include <GL/wglew.h>
#include <cstdio>

namespace lum {
	void setupPixelFormat(HDC hDC)
	{
		PIXELFORMATDESCRIPTOR pfd = {
			sizeof(PIXELFORMATDESCRIPTOR),
			1,                              // version
			PFD_SUPPORT_OPENGL |
			PFD_DRAW_TO_WINDOW |
			PFD_DOUBLEBUFFER,
			PFD_TYPE_RGBA,
			32,                             //color bits
			0, 0, 0, 0, 0, 0, 0, 0,         // ignored (more color bits)
			0, 0, 0, 0, 0,                  // ignored (accumulation)
			24,                             // depth buffer
			8,                              // stencil
			0,                              // aux
			0,								// layer (ignored)
			0,                              // reserved
			0, 0, 0,                        // masks
		};

		int pixelFormat = ChoosePixelFormat(hDC, &pfd);
		assert(pixelFormat && "ChoosePixelFormat failed");

		if (!SetPixelFormat(hDC, pixelFormat, &pfd)) {
			assert(false && "SetPixelFormat failed");
		}
	}

	HGLRC initDC(HWND hWnd, bool debug) {
		HDC hDC = GetDC(hWnd);

		// 1. Create non-debug context.
		lum::setupPixelFormat(hDC);
		HGLRC hglrc = wglCreateContext(hDC);
		if (!hglrc) {
			return hglrc;
		}
		wglMakeCurrent(hDC, hglrc);
		if (GLenum err = glGetError()) {
			printf("Foo\n");
		}

		int minor, major;
		glGetIntegerv(GL_MAJOR_VERSION, &major);
		glGetIntegerv(GL_MINOR_VERSION, &minor);
		printf("Default Context: Opengl Version %d.%d\n", major, minor);

		// 2. Init extension pointers
		if (glewInit() != GLEW_NO_ERROR) {
			MessageBoxA(0, "Failed to initialize glew", "OPENGL VERSION", 0);
			return 0;
		}

		// 3. Get Debug Context. Extension Pointers stay valid.
		int debug_bit = debug ? WGL_CONTEXT_DEBUG_BIT_ARB : 0;
		wglDeleteContext(hglrc);
		int attribs[] = { WGL_CONTEXT_FLAGS_ARB, debug_bit | WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
			WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
			WGL_CONTEXT_MAJOR_VERSION_ARB, 4,
			WGL_CONTEXT_MINOR_VERSION_ARB, 5,
			0 };
		HGLRC ret = wglCreateContextAttribsARB(hDC, 0, attribs);
		assert(ret);
		wglMakeCurrent(hDC, ret);

		glGetIntegerv(GL_MAJOR_VERSION, &major);
		glGetIntegerv(GL_MINOR_VERSION, &minor);
		printf("Debug Context: Opengl Version %d.%d\n", major, minor);

		ReleaseDC(hWnd, hDC);
		return ret;
	}

	void deleteWglContext(HGLRC hglrc) {
		BOOL success = wglDeleteContext(hglrc);
		assert(success && "wglDeleteContext failed");
	}
}