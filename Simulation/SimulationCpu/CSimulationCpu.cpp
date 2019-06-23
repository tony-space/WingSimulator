#include "pch.hpp"
#include "CSimulationCpu.hpp"

using namespace wing2d::simulation;
using namespace wing2d::simulation::serialization;
using namespace wing2d::simulation::cpu;

std::unique_ptr<ISimulation> wing2d::simulation::cpu::CreateSimulation()
{
	return std::make_unique<CSimulationCpu>();
}

void CSimulationCpu::ResetState(const SimulationState& state)
{
	m_state = state;
}

float CSimulationCpu::Update(float dt)
{
	for (auto& p : m_state.particles)
	{
		p.pos += p.vel * dt;
	}

	return dt;
}

void CSimulationCpu::GetState(SimulationState& outState)
{
	outState = m_state;
}