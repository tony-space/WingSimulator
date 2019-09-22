#pragma once

#include <functional>
#include <memory>
#include "../Simulation/Simulation.hpp"


#ifndef RENDERING_IMPL
#define RENDERING_API __declspec(dllimport)
#else
#define RENDERING_API __declspec(dllexport)
#endif // !RENDERING_IMPL


namespace wing2d
{
	namespace rendering
	{
		struct IRenderer
		{
			virtual void SetOnUpdate(std::function<void()> onUpdate) = 0;
			virtual void RenderAsync(const simulation::SimulationState& state) = 0;
			virtual void InitWindowLoop(size_t width, size_t height, bool fullscreen = false) = 0;
			virtual ~IRenderer() = default;
		};

		namespace opengl
		{
			std::unique_ptr<IRenderer> RENDERING_API CreateRenderer(int argc, char** argv);
		}
	}
}