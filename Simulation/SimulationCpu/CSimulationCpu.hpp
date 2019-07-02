#pragma once
#include "../Simulation.hpp"

#include "CLineSegment.hpp"
#include "CTriangle.hpp"

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
				virtual float Update() override;
				virtual const serialization::SimulationState& GetState() override;
			private:
				wing2d::simulation::serialization::SimulationState m_state;

				std::vector<CLineSegment> m_walls;
				std::vector<CTriangle> m_wing;

				void BuildWalls(const serialization::SimulationState &state);
				void BuildWing(const serialization::SimulationState &state);
			};
		}
	}
}