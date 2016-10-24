/*
copyright ARToolkit
copyright 2016: Lukas Murmann (don't use glut; use mediafoundation).
*/

#include <windows.h>

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <AR/param.h>
#include <AR/ar.h>

#include <Eigen/Dense>

#include "lum_gl.h"
#include "lum_glwindow.h"
#include "lum_mediasource.h"
#include "lum_quadrenderer.h"
#include "lum_util_gl.h"
#include "lum_glprogram2.h"
#include "lum_vertexrecorder.h"
#include "lum_win32tools.h"
#define MULTI
#ifdef MULTI
#include <AR/arMulti.h>
#endif

#define NOBENCHMARK
#define NOFLEN_HARDCODED

using namespace lum;

const vec3f COLOR_RED(1, 0, 0);
const vec3f COLOR_GREEN(0, 1, 0);
const vec3f COLOR_BLUE(0, 0, 1);
const vec3f COLOR_YELLOW(1, 1, 0);

lum::mediasource* g_msrc;
mat4f V;

int             xsize = 640;
int             ysize = 480;
int             thresh = 100;
int             count = 0;

ARParam         cparam;

#ifndef FLEN_HARDCODED
char                *cparam_name= "Data/lumwebcam.calib";
#endif

#ifdef MULTI
char                *config_name = "Data/multi/marker2x.dat";
ARMultiMarkerInfoT  *config;
#else
char           *patt_name = "Data/patt.hiro";
int             patt_id;
#endif
double          patt_width = 80.0;
double          patt_center[2] = { 0.0, 0.0 };
double          patt_trans[3][4];

static void   init(void);
static void   cleanup(void);
static void   mainLoop(void);

LPCSTR szWindowClass = "ARTestClass";
LPCSTR szTitle = "ARToolkit SimpleTest";
const wchar_t* c_symname = LR"RAWSTR(\\?\usb#vid_0458&pid_708c&mi_00#6&29a4ead7&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\global)RAWSTR";

HINSTANCE hInst;
HWND hWnd;

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
GLuint tex;


int main(int argc, char **argv)
{
    hInst = GetModuleHandle(NULL);
    MyRegisterClass(hInst);
    hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInst, NULL);

    bool debug = true;
    HGLRC hglrc = lum::initDC(hWnd, debug);
    lum::setupDebugPrint(stdout);

    init();


    if (!hWnd)
    {
        printf("Window Creation Failed\n");
        return -1;
    }

    // initialize remaining state
    ShowWindow(hWnd, SW_SHOW);

    g_msrc = new lum::mediasource();
    g_msrc->configure(xsize, ysize, c_symname);


    MSG msg;
    while (true) {
        while (PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                goto done;
            }
            DispatchMessage(&msg);
            InvalidateRect(hWnd, NULL, FALSE);
        }
    }

done:
    cleanup();
    wglDeleteContext(hglrc);
    return (int)msg.wParam;
}


static void setXformUniforms(GLuint program, const mat4f& M, const mat4f& V, const mat4f& P) {
    {
        int loc = glGetUniformLocation(program, "P");
        glUniformMatrix4fv(loc, 1, false, P.data());
    }

    {
        int loc = glGetUniformLocation(program, "V");
        glUniformMatrix4fv(loc, 1, false, V.data());
    }
    {
        int loc = glGetUniformLocation(program, "M");
        glUniformMatrix4fv(loc, 1, false, M.data());
    }
    {
        int loc = glGetUniformLocation(program, "N");
        glUniformMatrix4fv(loc, 1, false, M.inverse().transpose().data());
    }
}

static void drawScreenSpace(GLuint program, const ARMarkerInfo& info) {
    setXformUniforms(program, mat4f::Identity(), mat4f::Identity(), mat4f::Identity());
    {
        // draw in screen coordsl
        vec3f col[] = { COLOR_RED, COLOR_BLUE, COLOR_GREEN, COLOR_YELLOW };
        lum::VertexRecorder rec;
        mat4f P = lum::orthoMatrix((float)xsize, (float)ysize, 0.1f, 100.0f);
        vec3f pts[] = { vec3f(0.0f, 0.0f, 0.0f) ,
                        vec3f(0.5f, 0.0f, 0.0f),
                        vec3f(0.5f, 0.5f, 0.0f),
                        vec3f(0.0f, 0.5f, 0.0f) };
        for (int i = 0; i < 4; ++i) {
            const double* v = info.vertex[i];
            vec4f p((float)v[0], (float)v[1], 0.0f, 1);
            //vec4f p(pts[0][0], pts[0][1], 0.0f, 1);
            vec4f p1x = P * p;

            rec.record_poscolor(vec3f(p1x[0], p1x[1], 0.0), col[i]);
            //rec.record_poscolor(pts[i], col[i]);
        }

        glPointSize(5.0f);
        rec.draw(GL_POINTS);
    }
}
void drawWorldSpace(GLuint program, const ARMarkerInfo& info) {
    {
        mat4f P;
        argConvGLcpara2(&cparam, xsize, ysize, AR_GL_CLIP_NEAR, AR_GL_CLIP_FAR, P.data());
        setXformUniforms(program, mat4f::Identity(), V, P);
    }
    
    {
        int loc = glGetUniformLocation(program, "V");
        //glUniformMatrix4fv(loc, 1, false, V.data());
        glUniformMatrix4fv(loc, 1, false, V.data());
    }
    {
        mat4f M = mat4f::Identity();
        int loc = glGetUniformLocation(program, "M");
        glUniformMatrix4fv(loc, 1, false, M.data());
    }
    {
        mat4f N = mat4f::Identity();
        int loc = glGetUniformLocation(program, "N");
        glUniformMatrix4fv(loc, 1, false, N.data());
    }


    {
        lum::VertexRecorder recorder;
        vec3f UNITX = vec3f::UnitX();
        vec3f UNITY = vec3f::UnitY();
        vec3f UNITZ = vec3f::UnitZ();
        vec3f ORGN = vec3f(0.1f, 0, 0);

        const float s = 0.08f;
        const float sh = s / 2.0f;
        recorder.record_poscolor(ORGN, COLOR_RED);
        recorder.record_poscolor(ORGN + s * UNITX, COLOR_RED);

        recorder.record_poscolor(ORGN, COLOR_GREEN);
        recorder.record_poscolor(ORGN + s * UNITY, COLOR_GREEN);

        recorder.record_poscolor(ORGN, COLOR_BLUE);
        recorder.record_poscolor(ORGN + s * UNITZ, COLOR_BLUE);

        recorder.draw(GL_LINES);

        recorder.clear();
        recorder.record_poscolor(ORGN, vec3f(0, 1, 1));
        recorder.draw(GL_POINTS);
    }
}

#ifndef MULTI
int highest_confidence_match(ARMarkerInfo* marker_info, int marker_num) {
    int k = -1;
    for (int j = 0; j < marker_num; j++) {
        if (patt_id == marker_info[j].id) {
            if (k == -1) k = j;
            else if (marker_info[k].cf < marker_info[j].cf) k = j;
        }
    }
    return k;
}
#endif

/* main loop */
static void mainLoop(void)
{

    /* grab a vide frame */
    if (!g_msrc->grab()) {
        arUtilSleep(2);
        return;
    }
    int len = 0;
    ARUint8* dataPtr = (ARUint8*)g_msrc->getPtr(&len);
    if (!dataPtr) {
        arUtilSleep(2);
        return;
    }
    assert(len == xsize * ysize * 4);
    if (count == 0) arUtilTimerReset();
    count++;

    /* detect the markers in the video frame */
    int marker_num;
    ARMarkerInfo* marker_info;

    bool benchmark = false;
#ifdef BENCHMARK
    static int frame = 0;
    frame++;
    benchmark = frame % 60 == 0;
    lum::win32counter counter;
    counter.tic();
#endif

    if (arDetectMarkerLite(dataPtr, thresh, &marker_info, &marker_num) < 0) {
        cleanup();
        exit(0);
    }
    if (!marker_num) {
        return;
    }
#ifndef MULTI
    int k = highest_confidence_match(marker_info, marker_num);
    if (k == -1) {
        return;
    }
#endif

    if (!benchmark) {
        glTextureSubImage2D(tex, 0, 0, 0, xsize, ysize, GL_BGRA, GL_UNSIGNED_BYTE, dataPtr);
        lum::quadrenderer qr;
        qr.draw_with_tex(tex);
    }


#ifdef MULTI
    {
        double err;
        if ((err = arMultiGetTransMat(marker_info, marker_num, config)) < 0) {
            return;
        }
        if (err > 100.0) {
            printf("err = %f\n", err);
            return;
        }
    }
    double tmp[16];
    argConvGlpara(config->trans, tmp);
#else
    double tmp[16];
    arGetTransMat4x4(&marker_info[k], patt_center, patt_width, tmp);
#endif
    for (int i = 0; i < 16; ++i) {
        V.data()[i] = (float)tmp[i];
    }
    V = V * lum::scaleXYZ(1000.0f);
#ifdef BENCHMARK
    if (benchmark) {
        double elapsedms = counter.tocms();
        printf("Took %.1fms\n", (float)elapsedms);
    }
#endif BENCHMARK
    g_msrc->releasePtr();
    mat4f C = V.inverse();
    //printf("Z = %.2f\n", C(2, 3));

    std::string basepath("");
    std::string vshader = lum::readfile(basepath + "vshader.glsl");
    std::string fshader = lum::readfile(basepath + "fshader.glsl");
    GLuint program = lum::linkProgram(vshader.c_str(), fshader.c_str());
    if (!program) {
        return;
    }
    glUseProgram(program);
    for (int i = 0; i < marker_num; ++i) {
        drawScreenSpace(program, marker_info[i]);
    }
    if (marker_num) {
        drawWorldSpace(program, marker_info[0]);
    }
    glDeleteProgram(program);
}

static void init(void)
{
    /* open the video path */
    printf("Image size (x,y) = (%d,%d)\n", xsize, ysize);

#ifndef FLEN_HARDCODED
    ARParam wparam;
    if (arParamLoad(cparam_name, 1, &wparam) < 0) {
        printf("Camera parameter load error !!\n");
        exit(0);
    }
    arParamChangeSize(&wparam, xsize, ysize, &cparam);
#else
    cparam = {
        xsize = xsize,
        ysize = ysize
    };
    // fixme this is very approximate for webcam. use calibration.
    double fx = 700.95 * 0.45;
    double fy = fx;

    double mat_r0[] = { fx, 0, 316.5, 0 };
    double mat_r1[] = { 0, fy, 241.5, 0, 0 };
    double mat_r2[] = { 0, 0, 1, 0 };
    double dist_factor[] = { 318.5, 263.5, 26.2, 1.01275653 };
    // SIMPLIFICATION. Get Calibration values from toolbox.
    dist_factor[0] = xsize / 2;
    dist_factor[1] = ysize / 2;
    dist_factor[2] = 1.0;
    dist_factor[3] = 1.0;
    for (int i = 0; i < 4; ++i) {
        cparam.mat[0][i] = mat_r0[i];
        cparam.mat[1][i] = mat_r1[i];
        cparam.mat[2][i] = mat_r2[i];
        cparam.dist_factor[i] = dist_factor[i];
    }
#endif

    arInitCparam(&cparam); // setup global variables ...
    printf("*** Camera Parameter ***\n");
    arParamDisp(&cparam);

#ifdef MULTI
    if ((config = arMultiReadConfigFile(config_name)) == NULL) {
        printf("config data load error !!\n");
        exit(0);
    }
#else
    if ((patt_id = arLoadPatt(patt_name)) < 0) {
        printf("pattern load error !!\n");
        exit(0);
    }
#endif

    tex = lglCreateTexture(GL_TEXTURE_2D);
    glTextureParameteri(tex, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTextureStorage2D(tex, 1, GL_RGB8, xsize, ysize);
    assert(tex);
}

static void cleanup() {
    lglDeleteTexture(tex);
}

//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEX wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = NULL;
    wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wcex.lpszMenuName = NULL;
    wcex.lpszClassName = szWindowClass;
    wcex.hIconSm = NULL;

    return RegisterClassEx(&wcex);
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND	- process the application menu
//  WM_PAINT	- Paint the main window
//  WM_DESTROY	- post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    //int wmId, wmEvent;
    PAINTSTRUCT ps;
    HDC hdc;

    switch (message)
    {
    case WM_CREATE:
        break;
    case WM_COMMAND:
        break;
    case WM_PAINT:
    {
        hdc = BeginPaint(hWnd, &ps);
        // TODO: Add any drawing code here...

        RECT rect;
        GetClientRect(hWnd, &rect);
        int h = (int)(rect.bottom - rect.top);
        int w = (int)(rect.right - rect.left);
        glViewport(0, h - 480, 640, 480);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        mainLoop();

        SwapBuffers(hdc);
        EndPaint(hWnd, &ps);
        break;
    }
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    default:
        //if (g_win32input->handle_msg(hWnd, message, wParam, lParam)) {
            //break;
        //}
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

