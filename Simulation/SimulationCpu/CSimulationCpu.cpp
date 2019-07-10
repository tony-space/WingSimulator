#include "pch.hpp"
#include "CSimulationCpu.hpp"
#include "collisions.hpp"

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
	BuildWalls();
}

float CSimulationCpu::Update(float dt)
{
	collisions::SCollisionForecast closestCollision;
	closestCollision.timeToCollision = INFINITY;
	size_t particleId = -1;

	for (size_t i = 0; i < m_state.particles.size(); ++i)
	{
		const auto& particle = m_state.particles[i];

		for (const auto& point : m_state.airfoil)
		{
			collisions::SCollisionForecast response;
			if (collisions::TimeToPoint(particle.pos, particle.vel, m_state.particleRad, point, response))
			{
				if (response.timeToCollision < closestCollision.timeToCollision)
				{
					closestCollision = response;
					particleId = i;
				}
			}
		}
	}

	if (dt < closestCollision.timeToCollision)
		particleId = -1;
	else
		dt = closestCollision.timeToCollision;

	for (auto& p : m_state.particles)
	{
		p.pos += p.vel * dt;

		for (const auto& w : m_walls)
		{
			auto distance = w.DistanceToLine(p.pos) - m_state.particleRad;
			if (distance <= 0.0f && glm::dot(p.vel, w.normal()) < 0.0f)
			{
				p.vel = glm::reflect(p.vel, w.normal());
				p.pos -= w.normal() * distance;
			}
		}
	}

	if (particleId != -1)
	{
		auto& p = m_state.particles[particleId];
		p.vel = glm::reflect(p.vel, closestCollision.normal);
	}

	return dt;
}

const serialization::SimulationState& CSimulationCpu::GetState()
{
	return m_state;
}

void CSimulationCpu::BuildWalls()
{
	glm::vec2 corner(m_state.worldSize.width / 2.0f, m_state.worldSize.height / 2.0f);
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