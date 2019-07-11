#pragma once
#include "../Simulation.hpp"

#include "CLineSegment.hpp"

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
				virtual const serialization::SimulationState& GetState() override;
			private:
				wing2d::simulation::serialization::SimulationState m_state;
				std::vector<CLineSegment> m_walls;
				std::vector<glm::vec2> m_forces;

				size_t m_fixedCount = 0;

				void BuildWalls();
				void BuildWing();
			};
		}
	}
}