#include "pch.hpp"
#include "CSimulationCpu.hpp"
#include "CBoundingBox.hpp"

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

	m_wing.clear();
	m_wing.reserve(state.wing.triangles.size());

	auto range = state.wing.triangles | boost::adaptors::transformed([&](const auto& t)
	{
		const glm::vec2& a = state.wing.airfoil[t.i1];
		const glm::vec2& b = state.wing.airfoil[t.i2];
		const glm::vec2& c = state.wing.airfoil[t.i3];

		return CTriangle(a, b, c);
	});

	boost::range::push_back(m_wing, range);
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

const serialization::SimulationState& CSimulationCpu::GetState()
{
	return m_state;
}
