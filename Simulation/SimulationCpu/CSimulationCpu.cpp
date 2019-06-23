#include "pch.hpp"
#include "CSimulationCpu.hpp"

std::unique_ptr<wing2d::ISimulation> wing2d::cpu::CreateSimulation()
{
	return std::make_unique<wing2d::cpu::CSimulationCpu>();
}

void wing2d::cpu::CSimulationCpu::ResetState(const serialization::SimulationState& state)
{
	m_state = state;
}

float wing2d::cpu::CSimulationCpu::Update(float dt)
{
	for (auto& p : m_state.particles)
	{
		p.pos += p.vel * dt;
	}

	return dt;
}

void wing2d::cpu::CSimulationCpu::GetState(serialization::SimulationState& outState)
{
	outState = m_state;
}