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

#ifndef LUM_MOUSEINPUT_H
#define LUM_MOUSEINPUT_H
#include <Windows.h>
#include <functional>
namespace lum {
typedef std::function<void(char k)> keycb;
typedef std::function<void(float x, float y, float s)> vscrollcb;
typedef std::function<void(float x, float y)> clickcb;
typedef std::function<void(float x, float y, float dx, float dy, int modkeys)> movecb;

const int MODKEY_LSHIFT = 1;
const int MODKEY_LCTRL = 2;
const int MODKEY_LCLICK = 3;
const int MODKEY_RCLICK = 4;

class win32input{
public:
	win32input();
	bool handle_msg(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

	clickcb m_clickcb;
	movecb m_movecb;
	vscrollcb m_vscrollcb;
	keycb m_keycb;

private:
	POINT m_scroll;
	bool m_scrolling;
};

struct win32counter {
	win32counter();
	LONGLONG now();
	void tic();
	float tocms();

	// counts per second
	LONGLONG freq;
	// counts since process startup
	LONGLONG start;
};
}
#endif