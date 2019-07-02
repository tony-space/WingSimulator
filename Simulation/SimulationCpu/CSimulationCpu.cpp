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

	BuildWing(state);
	BuildWalls(state);
}

float CSimulationCpu::Update()
{
	constexpr float dt = 0.0001f;

	for (auto& p : m_state.particles)
	{
		p.pos += p.vel * dt;

		for (const auto& w : m_walls)
		{
			auto distance = w.DistanceToLine(p.pos) - m_state.particleRad;
			if (distance <= 0.0f && glm::dot(p.vel, w.normal()) < 0.0f)
				p.vel = glm::reflect(p.vel, w.normal());
		}
	}


	return dt;
}

const serialization::SimulationState& CSimulationCpu::GetState()
{
	return m_state;
}

void CSimulationCpu::BuildWing(const SimulationState &state)
{
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

void CSimulationCpu::BuildWalls(const SimulationState &state)
{
	glm::vec2 corner(state.worldSize.width / 2.0f, state.worldSize.height / 2.0f);
	glm::vec2 topLeft(-corner.x, corner.y);
	glm::vec2 topRight(corner.x, corner.y);
	glm::vec2 bottomRight(corner.x, -corner.y);
	glm::vec2 bottomLeft(-corner.x, -corner.y);

	m_walls.clear();
	m_walls.reserve(4);
	m_walls.emplace_back(topLeft, topRight);
	m_walls.emplace_back(topRight, bottomRight);
	m_walls.emplace_back(bottomRight, bottomLeft);
	m_walls.emplace_back(bottomLeft, topLeft);
}
