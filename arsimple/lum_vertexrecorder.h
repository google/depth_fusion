#ifndef RECORDER_H
#define RECORDER_H

#include <vector>
#include "lum_gl.h"
#include "lum_transform.h"

namespace lum {
class VertexRecorder {
public:
    VertexRecorder();
    // write a vertex into the CPU buffer
    void record(vec3f pos,
        vec3f normal);
    void record(vec3f pos,
        vec3f normal,
        vec3f color);
    void record_poscolor(vec3f pos,
        vec3f color);
    // draw recorded points
    void draw(GLenum mode = GL_TRIANGLES);
    // empties the recording buffer.
    void clear();
private:
    int m_nverts;
    std::vector<vec3f> m_position;
    std::vector<vec3f> m_normal;
    std::vector<vec3f> m_color;
};

// draw a sphere with radius r centered at (0,0,0)
// slices and stacks control the level of detail of the sphere
void drawSphere(float r, int slices, int stacks);

// draw a cylinder. the cylinder extends from y=0 to y=h
// and from -r to +r in the XZ plane.
void drawCylinder(int nsides, float r, float h);

// draw a quad in the XZ plane with normal in +Y direction
void drawQuad(float w);
}

#endif
