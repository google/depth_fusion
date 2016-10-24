#version 330
in vec4 var_Color;
in vec3 var_Normal;
in vec3 var_Position;

layout(location=0) out vec4 out_Color;
uniform mat4 projP;
uniform mat4 projV;
uniform sampler2D projTex;

void main() {
	vec4 uv = projP * projV * vec4(var_Position, 1);
	uv /= uv.w;
	uv = (uv + 1) / 2;
	uv.y = 1 - uv.y;
	if (uv.x > 0 && uv.x < 1 &&
	    uv.y > 0 && uv.y < 1 
	    /*uv.z > 0 && uv.z < 1 */) {
	    out_Color = vec4(uv.xy, 0, 1);
		out_Color = texture(projTex, uv.xy);
	} else {
		out_Color = var_Color;
		out_Color = vec4(1, 0, 0, 1);
	}
}