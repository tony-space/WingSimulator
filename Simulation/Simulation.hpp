#pragma once

#include <vector>
#include <memory>
#include <cstdint>
#include <glm/vec2.hpp>

namespace wing2d
{
	namespace simulation
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
					float width = 2.0f;
					float height = 2.0f;
				} worldSize;

				float particleRad = 0.01f;
				std::vector<Particle> particles;
				struct SWing
				{
					struct STriangle
					{
						using value_type = uint16_t;
						value_type i1;
						value_type i2;
						value_type i3;
					};

					std::vector<glm::vec2> airfoil;
					std::vector<STriangle> triangles;
				} wing;
			};
		}

		struct ISimulation
		{
			virtual void ResetState(const serialization::SimulationState& state) = 0;
			virtual float Update(float dt) = 0;
			virtual const serialization::SimulationState& GetState() = 0;
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
}