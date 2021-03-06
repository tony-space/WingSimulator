#pragma once
#include <glm/glm.hpp>

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
				GLuint m_vao = -1;
				GLuint m_vboPos = -1;
				GLuint m_vboColor = -1;
				std::unique_ptr<CShaderProgram> m_program;

				//x - width, y - height, z - aspect ratio
				glm::vec3 m_window;

				std::vector<glm::vec2> m_pos;
				std::vector<glm::vec4> m_color;

				void InitScene();

				static COpenGLRenderer* instance;
				static void DisplayFunc();
				static void ReshapeFunc(int w, int h);
				static void CloseFunc();
			};
		}
	}
}
