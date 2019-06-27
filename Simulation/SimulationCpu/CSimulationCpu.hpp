#pragma once
#include "../Simulation.hpp"

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CSimulationCpu : public ISimulation
			{
			public:
				virtual ~CSimulationCpu() override = default;
				virtual void ResetState(const serialization::SimulationState& state) override;
				virtual float Update(float dt) override;
				virtual void GetState(serialization::SimulationState& outState) override;
			private:
				wing2d::simulation::serialization::SimulationState m_state;

				//struct SLine
				//{
				//	glm::vec2 origin;
				//	glm::vec2 ray;
				//	glm::vec2 normal;
				//	float length;
				//	float __padding;

				//	SLine() = default;
				//	SLine(const glm::vec2& v1, const glm::vec2& v2);
				//	bool Intersected(const glm::vec2& spherePos, float sphereRad, glm::vec2& contactNorm, float& outDepth);
				//};
			};
		}
	}
}