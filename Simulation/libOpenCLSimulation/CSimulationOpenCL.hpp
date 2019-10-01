#pragma once

#include "../Simulation.hpp"

namespace wing2d
{
	namespace simulation
	{
		namespace opencl
		{
			class CSimulationOpenCL : public ISimulation
			{
				virtual void ResetState(const SimulationState& state) override;
				virtual float Update(float dt) override;
				virtual const SimulationState& GetState() override;

			private:

				SimulationState m_state;
			};
		}
	}
}