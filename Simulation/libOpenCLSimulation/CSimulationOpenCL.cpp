#include "pch.hpp"

#include "CSimulationOpenCL.hpp"

using namespace wing2d::simulation;
using namespace wing2d::simulation::opencl;

std::unique_ptr<ISimulation> wing2d::simulation::opencl::CreateSimulation()
{
	return std::make_unique<CSimulationOpenCL>();
}

void CSimulationOpenCL::ResetState(const SimulationState& state)
{
}

float CSimulationOpenCL::Update(float dt)
{
	return 0.0f;
}

const SimulationState& CSimulationOpenCL::GetState()
{
	return m_state;
}
