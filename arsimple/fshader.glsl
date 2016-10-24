#version 330
in vec4 var_Color;
in vec3 var_Normal;
in vec3 var_Position;

layout(location=0) out vec4 out_Color;

void main() {
    out_Color = var_Color;
}