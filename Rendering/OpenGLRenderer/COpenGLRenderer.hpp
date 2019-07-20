#pragma once
#include "../Rendering.hpp"
#include "CShaderProgram.hpp"

namespace wing2d
{
	namespace rendering
	{
		namespace opengl
		{
			class COpenGLRenderer : public IRenderer
			{
			public:
				COpenGLRenderer();
				virtual ~COpenGLRenderer() override;

				virtual void SetOnUpdate(std::function<void()> onUpdate) override;
				virtual void RenderAsync(const simulation::SimulationState& state) override;
				virtual void InitWindowLoop(size_t width, size_t height, bool fullscreen = false) override;

			private:
				std::function<void()> m_onUpdate;
				GLuint m_vao;
				GLuint m_vboPos;
				GLuint m_vboColor;
				std::unique_ptr<CShaderProgram> m_program;

				//x - width, y - height, z - aspect ratio
				glm::vec3 m_window;

				void InitScene();

				static COpenGLRenderer* instance;
				static void DisplayFunc();
				static void ReshapeFunc(int w, int h);
				static void CloseFunc();
			};
		}
	}
}
