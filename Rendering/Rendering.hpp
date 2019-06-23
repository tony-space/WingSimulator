#pragma once

#include <functional>
#include <memory>
#include "../Simulation/Simulation.hpp"

namespace wing2d
{
	namespace rendering
	{
		struct IRenderer
		{
			virtual void SetOnUpdate(std::function<void()> onUpdate) = 0;
			virtual void RenderAsync(const simulation::serialization::SimulationState& state) = 0;
			virtual void Synchronize() = 0;
			virtual void InitWindowLoop(size_t width, size_t height, bool fullscreen = false) = 0;
			virtual ~IRenderer() = default;
		};

		namespace opengl
		{
			std::unique_ptr<IRenderer> CreateRenderer(int argc, char** argv);
		}
	}
}