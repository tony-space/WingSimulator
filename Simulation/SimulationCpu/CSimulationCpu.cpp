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
	float left = -m_state.worldSize.width * 0.5f;
	float right = m_state.worldSize.width * 0.5f;

	float bottom = -m_state.worldSize.height * 0.5f;
	float top = m_state.worldSize.height * 0.5f;

	for (auto& p : m_state.particles)
	{
		p.pos += p.vel * dt;
		if (p.pos.x + m_state.particleRad > right)
		{
			p.pos.x = right - m_state.particleRad;
			p.vel.x = -glm::abs(p.vel.x);
		}

		if (p.pos.x - m_state.particleRad < left)
		{
			p.pos.x = left + m_state.particleRad;
			p.vel.x = glm::abs(p.vel.x);
		}

		if (p.pos.y + m_state.particleRad > top)
		{
			p.pos.y = top - m_state.particleRad;
			p.vel.y = -glm::abs(p.vel.y);
		}

		if (p.pos.y - m_state.particleRad < bottom)
		{
			p.pos.y = bottom + m_state.particleRad;
			p.vel.y = glm::abs(p.vel.y);
		}
	}

	return dt;
}

void CSimulationCpu::GetState(SimulationState& outState)
{
	outState = m_state;
}