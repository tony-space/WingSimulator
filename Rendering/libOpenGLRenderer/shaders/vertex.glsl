#version 120
uniform float radius;

in vec2 pos;
in vec4 color;

void main()
{
	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_PointSize = radius;
	gl_FrontColor = color;
	gl_Position = gl_ModelViewProjectionMatrix * vec4(pos, 0.0, 1.0);
}