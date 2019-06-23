#pragma once

#include <vector>
#include <memory>
#include <glm/vec2.hpp>

namespace wing2d
{
	namespace serialization
	{
		struct Particle
		{
			glm::vec2 pos;
			glm::vec2 vel;
		};

		struct SimulationState
		{
			struct
			{
				float left = -1.0f;
				float right = 1.0f;

				float bottom = -1.0f;
				float top = 1.0f;
			} worldBoundaries;

			std::vector<Particle> particles;
			std::vector<glm::vec2> wing;
		};
	}

	struct ISimulation
	{
		virtual void ResetState(const serialization::SimulationState& state) = 0;
		virtual float Update(float dt) = 0;
		virtual void GetState(serialization::SimulationState& outState) = 0;
		virtual ~ISimulation() = default;
	};

	namespace cpu
	{
		std::unique_ptr<ISimulation> CreateSimulation();
	}

	namespace cuda
	{
		std::unique_ptr<ISimulation> CreateSimulation();
	}
}