#pragma once

#include <memory>
#include <thrust/device_malloc.h>

#include "../Simulation.hpp"
#include "CudaInterfaces.cuh"

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			class CSimulationCuda : public ISimulation
			{
			public:
				virtual void ResetState(const SimulationState& state) override;
				virtual float Update(float dt) override;
				virtual const SimulationState& GetState() override;

			private:
				OdeState_t m_curOdeState;
				OdeState_t m_nextOdeState;
				
				SimulationState m_state;

				std::unique_ptr<IOdeSolver> m_odeSolver;

				thrust::device_ptr<float> m_dt = thrust::device_malloc<float>(1);
				
				thrust::device_vector<float4> m_deviceColors;
				PinnedHostVector4D_t m_hostColors;

				PinnedHostVector2D_t m_hostOdeState;

				void CopyToGPU();
				void ColorParticles();
			};
		}
	}
}
