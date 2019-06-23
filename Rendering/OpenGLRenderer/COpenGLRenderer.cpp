#include "pch.hpp"
#include "COpenGLRenderer.hpp"

using namespace wing2d;
using namespace wing2d::rendering;
using namespace wing2d::rendering::opengl;
using namespace wing2d::simulation;
using namespace wing2d::simulation::serialization;

std::unique_ptr<IRenderer> wing2d::rendering::opengl::CreateRenderer(int argc, char** argv)
{
	glutInit(&argc, argv);
	//glewInit();
	return std::make_unique<COpenGLRenderer>();
}

void wing2d::rendering::opengl::COpenGLRenderer::SetOnUpdate(std::function<void() > onUpdate)
{
	throw std::logic_error("The method or operation is not implemented.");
}

void wing2d::rendering::opengl::COpenGLRenderer::RenderAsync(const simulation::serialization::SimulationState& state)
{
	throw std::logic_error("The method or operation is not implemented.");
}

void wing2d::rendering::opengl::COpenGLRenderer::Synchronize()
{
	throw std::logic_error("The method or operation is not implemented.");
}

void wing2d::rendering::opengl::COpenGLRenderer::InitWindowLoop(size_t width, size_t height, bool fullscreen /*= false*/)
{
	throw std::logic_error("The method or operation is not implemented.");
}
