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
	return std::make_unique<COpenGLRenderer>();
}

COpenGLRenderer::COpenGLRenderer()
{
	if (instance)
		throw std::domain_error("OpenGL renderer has been already instantiated");

	instance = this;
}

COpenGLRenderer::~COpenGLRenderer()
{
	instance = nullptr;
}

void COpenGLRenderer::SetOnUpdate(std::function<void()> onUpdate)
{
	m_onUpdate = onUpdate;
}

void COpenGLRenderer::InitScene()
{
	GLenum glError;

	//glEnable(GL_DEPTH_TEST);
	//glEnable(GL_CULL_FACE);

	//See the link below to read more  about GL_POINT_SPRITE_ARB
	//https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_point_sprite.txt
	glEnable(GL_POINT_SPRITE);
	glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

	m_program = std::make_unique<CShaderProgram>("shaders/vertex.glsl", "shaders/fragment.glsl");
	auto posLocation = m_program->GetAttributeLocation("pos");
	auto colorLocation = m_program->GetAttributeLocation("color");

	glGenVertexArrays(1, &m_vao);
	glGenBuffers(1, &m_vboPos);
	glGenBuffers(1, &m_vboColor);

	glBindVertexArray(m_vao);
	glBindBuffer(GL_ARRAY_BUFFER, m_vboPos);
	glEnableVertexAttribArray(posLocation);
	glVertexAttribPointer(posLocation, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

	glBindBuffer(GL_ARRAY_BUFFER, m_vboColor);
	glEnableVertexAttribArray(colorLocation);
	glVertexAttribPointer(colorLocation, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glError = glGetError();
	assert(glError == GL_NO_ERROR);
}

void COpenGLRenderer::CloseFunc()
{
	glDeleteBuffers(1, &instance->m_vboPos);
	glDeleteBuffers(1, &instance->m_vboColor);
	glDeleteVertexArrays(1, &instance->m_vao);
	instance->m_program.reset();

	assert(glGetError() == GL_NO_ERROR);
}

void COpenGLRenderer::DisplayFunc()
{
	instance->m_onUpdate();

	glutSwapBuffers();
	glutPostRedisplay();
	glutReportErrors();
}

void COpenGLRenderer::RenderAsync(const SimulationState& state)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	std::vector<glm::vec2> positions(state.particles.size());
	std::vector<glm::vec4> colors(state.particles.size());

	std::transform(state.particles.cbegin(), state.particles.cend(), positions.begin(), [](const auto& p)
	{
		return p.pos;
	});

	std::transform(state.particles.cbegin(), state.particles.cend(), colors.begin(), [](const auto& p)
	{
		//return glm::vec4(1.0f, 1.0f, 0.0f, 1.0f);
		return p.color;
	});

	glBindBuffer(GL_ARRAY_BUFFER, m_vboPos);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * positions.size(), positions.data(), GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, m_vboColor);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec4) * colors.size(), colors.data(), GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	{
		glBindVertexArray(m_vao);
		auto switcher = m_program->Activate();

		float pixelsPerUnit = m_window.y * 0.5f;

		m_program->SetUniform("radius", state.particleRad * pixelsPerUnit * 2.0f);
		glDrawArrays(GL_POINTS, 0, GLsizei(state.particles.size()));
		glBindVertexArray(0);
	}

	//glColor3f(0.25f, 0.25f, 0.25f);
	//glBegin(GL_LINE_LOOP);
	//for (const auto& v : state.airfoil)
	//	glVertex2fv(glm::value_ptr(v));
	//glEnd();

	//glColor3f(1.0f, 1.0f, 1.0f);
	//glBegin(GL_POINTS);
	//for (const auto& v : state.airfoil)
	//	glVertex2fv(glm::value_ptr(v));
	//glEnd();


	glFlush();
	assert(glGetError() == GL_NO_ERROR);
}

void COpenGLRenderer::ReshapeFunc(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	GLdouble aspectRatio = GLdouble(w) / h;
	glOrtho(-aspectRatio, aspectRatio, -1.0, 1.0, -1.0, 1.0);

	instance->m_window = glm::vec3(w, h, aspectRatio);
}

void COpenGLRenderer::InitWindowLoop(size_t width, size_t height, bool fullscreen /*= false*/)
{
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	if (fullscreen)
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_BORDERLESS | GLUT_CAPTIONLESS);
	else
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

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
