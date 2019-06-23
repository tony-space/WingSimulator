#include "pch.hpp"
#include "COpenGLRenderer.hpp"

using namespace wing2d;
using namespace wing2d::rendering;
using namespace wing2d::rendering::opengl;
using namespace wing2d::simulation;
using namespace wing2d::simulation::serialization;

COpenGLRenderer* COpenGLRenderer::instance = nullptr;

std::unique_ptr<IRenderer> wing2d::rendering::opengl::CreateRenderer(int argc, char** argv)
{
	glutInit(&argc, argv);
	//glewInit();
	return std::make_unique<COpenGLRenderer>();
}

COpenGLRenderer::COpenGLRenderer()
{
	if (instance)
		throw std::domain_error("OpenGL renderer has been already instantiated");

	instance = this;
}

void COpenGLRenderer::SetOnUpdate(std::function<void()> onUpdate)
{
	m_onUpdate = onUpdate;
}

void COpenGLRenderer::RenderAsync(const SimulationState& state)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	for (const auto p : state.particles)
	{
		glPushMatrix();
		glTranslatef(p.pos.x, p.pos.y, 0.0f);
		glutWireCube(state.particleRad);
		glPopMatrix();
	}

	glFlush();
}

void COpenGLRenderer::InitScene()
{
	glEnable(GL_DEPTH_TEST);

	//See the link below to read more  about GL_POINT_SPRITE_ARB
	//https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_point_sprite.txt
	glEnable(GL_POINT_SPRITE);
	glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
}

void COpenGLRenderer::DisplayFunc()
{
	instance->m_onUpdate();

	glutSwapBuffers();
	glutPostRedisplay();
	glutReportErrors();
}

void COpenGLRenderer::ReshapeFunc(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	GLdouble aspectRatio = GLdouble(w) / h;
	glOrtho(-aspectRatio, aspectRatio, -1.0, 1.0, -1.0, 1.0);
}

void COpenGLRenderer::CloseFunc()
{
}

void COpenGLRenderer::InitWindowLoop(size_t width, size_t height, bool fullscreen /*= false*/)
{
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	if (fullscreen)
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_BORDERLESS | GLUT_CAPTIONLESS);
	else
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);

	glutInitWindowSize(int(width), int(height));

	auto wnd = glutCreateWindow("OpenGL renderer");

	glutReshapeFunc(ReshapeFunc);
	glutDisplayFunc(DisplayFunc);
	glutCloseFunc(CloseFunc);

	auto err = glewInit();
	if (err)
	{
		glutDestroyWindow(wnd);
		throw std::runtime_error("glewInit() failed");
	}

	wglSwapIntervalEXT(0);
	InitScene();

	glutMainLoop();
}
