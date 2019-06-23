#pragma once
#include "../Rendering.hpp"

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
				virtual ~COpenGLRenderer() override = default;

				virtual void SetOnUpdate(std::function<void()> onUpdate) override;
				virtual void RenderAsync(const simulation::serialization::SimulationState& state) override;
				virtual void InitWindowLoop(size_t width, size_t height, bool fullscreen = false) override;

			private:
				std::function<void()> m_onUpdate;
				void InitScene();


				static COpenGLRenderer* instance;
				static void DisplayFunc();
				static void ReshapeFunc(int w, int h);
				static void CloseFunc();
			};
		}
	}
}
