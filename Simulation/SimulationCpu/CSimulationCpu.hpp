#pragma once
#include "../Simulation.hpp"

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
				virtual float Update(float dt) override;
				virtual const serialization::SimulationState& GetState() override;
			private:
				wing2d::simulation::serialization::SimulationState m_state;

				std::vector<CTriangle> m_wing;
			};
		}
	}
}