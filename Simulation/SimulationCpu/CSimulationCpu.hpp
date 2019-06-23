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
			};
		}
	}
}