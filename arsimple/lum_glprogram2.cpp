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

#include "lum_glprogram2.h"
#include "lum_gl.h"
#include <cstdio>
#include <fstream>

namespace lum {
bool compileStatus(GLuint shader) {
    int ret;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ret);
    return ret;
}

bool linkStatus(GLuint program) {
    int ret;
    glGetProgramiv(program, GL_LINK_STATUS, &ret);
    return ret != 0;
}

bool compileShader(GLuint handle, GLenum stype, const char* src) {
    int shader_len = (int)strlen(src);
    glShaderSource(handle, 1, &src, &shader_len);
    glCompileShader(handle);
    if (!compileStatus(handle)) {
        char buff[2048];
        int nwritten;
        glGetShaderInfoLog(handle, 2048, &nwritten, buff);

        const char* typelabel = stype == GL_VERTEX_SHADER ? "vertex" : (stype == GL_FRAGMENT_SHADER ? "fragment" : "unknown");
        printf("Error in %s shader\n%s\n", typelabel, buff);
        return false;
    }
    return true;
}

int compileShader(GLenum type, const char* src) {
    GLuint handle = glCreateShader(type);
    compileShader(handle, type, src);
    return handle;
}

bool linkProgram(GLuint handle, GLuint vshader, GLuint fshader) {
    glAttachShader(handle, vshader);
    glAttachShader(handle, fshader);
    glLinkProgram(handle);
    if (!linkStatus(handle)) {
        char buff[2048];
        int nwritten;
        glGetProgramInfoLog(handle, 2048, &nwritten, buff);
        printf("Program link error:\n%s\n", buff);
        return false;
    }
    return true;
}
bool linkProgram(GLuint handle, GLuint cshader) {
    glAttachShader(handle, cshader);
    glLinkProgram(handle);
    if (!linkStatus(handle)) {
        char buff[2048];
        int nwritten;
        glGetProgramInfoLog(handle, 2048, &nwritten, buff);
        printf("Program link error:\n%s\n", buff);
        return false;
    }
    return true;
}

uint32_t linkProgram(const std::string& vshader, const std::string& fshader) {
    return linkProgram(vshader.c_str(), fshader.c_str());
}
uint32_t linkProgram(const char* vshader_src, const char* fshader_src) {
    GLuint program = glCreateProgram();
    GLuint vshader = compileShader(GL_VERTEX_SHADER, vshader_src);
    GLuint fshader = compileShader(GL_FRAGMENT_SHADER, fshader_src);
    if (!linkProgram(program, vshader, fshader)) {
        glDeleteProgram(program);
        program = 0;
    }
    glDeleteShader(vshader);
    glDeleteShader(fshader);
    return program;
}
uint32_t linkProgram(const char* cshader_src) {
    GLuint program = glCreateProgram();
    GLuint cshader = compileShader(GL_COMPUTE_SHADER, cshader_src);
    if (!linkProgram(program, cshader)) {
        glDeleteProgram(program);
        program = 0;
    }
    glDeleteShader(cshader);
    return program;
}
uint32_t programFromFiles(const char* vsfile, const char* fsfile) {
    std::string vs = readfile(vsfile);
    std::string fs = readfile(fsfile);
    return linkProgram(vs.c_str(), fs.c_str());
}

std::string readfile(const std::string& fname) {
    std::ifstream instr(fname);
    if (!instr) {
        printf("Cannot read file %s\n", fname.c_str());
        return "";
    }
    std::string cppstr{ std::istreambuf_iterator < char > {instr},
        std::istreambuf_iterator < char > {} };
    return cppstr;
}
}