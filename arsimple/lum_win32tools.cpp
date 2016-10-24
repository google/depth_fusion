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

#include "lum_win32tools.h"
#include <cstdint>
#include <cstdio>

namespace lum {

win32input::win32input() : m_scrolling(false) {
	m_scroll.x = m_scroll.y = 0;
	if (!EnableMouseInPointer(TRUE)) {
		printf("Error, EnableMouseInPointer() failed\n");
	}
}

int getmodmask(WPARAM wParam) {
	int mask = 0;
	if (uint8_t shift = (uint8_t)GetKeyState(VK_LSHIFT)) {
		if (shift >> 7) { // low bit set means toggled
			mask |= MODKEY_LSHIFT;
		}
	}
	if (uint8_t ctrl = (uint8_t)GetKeyState(VK_LCONTROL)) {
		if (ctrl >> 7) { // low bit set means toggled
			mask |= MODKEY_LCTRL;
		}
	}
	bool lclick = IS_POINTER_FIRSTBUTTON_WPARAM(wParam);
	if (lclick) {
		mask |= MODKEY_LCLICK;
	}
	bool rclick = IS_POINTER_SECONDBUTTON_WPARAM(wParam);
	if (rclick) {
		mask |= MODKEY_RCLICK;
	}
	return mask;
}
bool win32input::handle_msg(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {
	switch (message) {
	case WM_POINTERDOWN:
	{
		int mask = getmodmask(wParam);
		if (mask & (MODKEY_LCLICK | MODKEY_RCLICK)) {
			POINT pt = { LOWORD(lParam), HIWORD(lParam) };
			ScreenToClient(hWnd, &pt);
			m_scroll = pt;
			m_scrolling = false;
		}
		return true;
	}
	case WM_POINTERUPDATE:
	{
		POINT pt = { LOWORD(lParam), HIWORD(lParam) };
		ScreenToClient(hWnd, &pt);
		float xdiff = (float)(pt.x - m_scroll.x);
		float ydiff = (float)(pt.y - m_scroll.y);


		int mask = getmodmask(wParam);
		if (mask & (MODKEY_LCLICK | MODKEY_RCLICK)) {
			m_scroll = pt;
			if (m_movecb) {
				m_movecb((float)pt.x, (float)pt.y, xdiff, ydiff, mask);
			}
			m_scrolling = true;
		}
		return true;
	}
	case WM_POINTERUP:
	{
		POINT pt = { LOWORD(lParam), HIWORD(lParam) };
		ScreenToClient(hWnd, &pt);
		if (!m_scrolling && m_clickcb) {
			m_clickcb((float)pt.x, (float)pt.y);
		}
	}
	return true;
	case WM_MOUSEWHEEL:
	{
		POINT pt = { LOWORD(lParam), HIWORD(lParam) };
		ScreenToClient(hWnd, &pt);
		if (m_vscrollcb) {
			int16_t mv = HIWORD(wParam);
			m_vscrollcb((float)pt.x, (float)pt.y, mv / 120.0f);
		}
	}
	break;
	case WM_KEYDOWN: {
		if (m_keycb) {
			m_keycb((char)wParam);
		}
		return true;
	}
	default:
		return false;
	}
	return true;
}
win32counter::win32counter() {
	if (!QueryPerformanceFrequency((LARGE_INTEGER*)&freq)) {
		printf("Cannot get performance frequency\n");
	}
}
LONGLONG win32counter::now() {
	LARGE_INTEGER n;
	if (!QueryPerformanceCounter(&n)) {
		printf("Cannot get performance counter\n");
	}
	return n.QuadPart;
}
void win32counter::tic() {
	start = now();
}
float win32counter::tocms() {
	LONGLONG n = now();
	return 1000.0f * (float)(n - start) / freq;
}
}