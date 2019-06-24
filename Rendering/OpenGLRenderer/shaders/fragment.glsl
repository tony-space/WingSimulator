#version 120

void main()
{
	vec2 pixelPos = gl_TexCoord[0].xy * vec2(2.0, -2.0) + vec2(-1.0, 1.0);
	float lenSq = dot(pixelPos, pixelPos);
	if (lenSq > 1.0) discard;
	gl_FragColor = gl_Color;
}