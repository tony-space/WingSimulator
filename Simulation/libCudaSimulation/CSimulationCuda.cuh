#pragma once

#include <memory>
#include <thrust/device_malloc.h>
#include <thrust/device_vector.h>

#include "../Simulation.hpp"
#include "CudaInterfaces.cuh"
#include "CDerivativeSolver.cuh"


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
				std::unique_ptr<CDerivativeSolver> m_derivativeSolver;
				OdeState_t m_curOdeState;
				OdeState_t m_nextOdeState;
				
				SimulationState m_state;

				std::unique_ptr<IOdeSolver> m_odeSolver;

				struct
				{
					thrust::device_vector<float> input;
					thrust::device_vector<uint8_t> cubInternalStorage;
					thrust::device_ptr<float> dt = thrust::device_malloc<float>(1);
				} m_minDeltaTime;
				
				thrust::device_vector<float4> m_deviceColors;
				thrust::device_vector<float4> m_deviceColors2;
				PinnedHostVector4D_t m_hostColors;

				PinnedHostVector2D_t m_hostOdeState;

				void CopyToGPU();
				void ColorParticles();
				void ColorParticles2();
				const thrust::device_ptr<float>& ComputeMinDeltaTime(float requestedDt);
			};
		}
	}
}
