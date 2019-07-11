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
	m_walls.clear();
	BuildWalls();
	BuildWing();

	m_forces.resize(m_state.particles.size() - m_fixedCount);
}

glm::vec2 SpringDamper(const glm::vec2& normal, const glm::vec2& vel1, const glm::vec2& vel2, float springLen)
{
	constexpr float stiffness = 10000.0f;
	constexpr float damp = 50.0f;
	auto v = glm::dot(vel1 - vel2, normal);
	auto force = normal * (springLen * stiffness + v * damp);
	return force;
}

float CSimulationCpu::Update(float dt)
{
	auto timeRange = m_state.particles | boost::adaptors::sliced(m_fixedCount, m_state.particles.size()) | boost::adaptors::transformed([&](const auto& p)
	{
		auto halfRadDt = m_state.particleRad * 0.5f / glm::length(p.vel);
		return std::min(halfRadDt, dt);
	});

	dt = std::reduce(timeRange.begin(), timeRange.end(), dt, [](auto t1, auto t2) {return t1 < t2 ? t1 : t2; });

	std::fill(m_forces.begin(), m_forces.end(), glm::vec2(0.0f));

	auto diameter = m_state.particleRad * 2.0f;
	auto diameterSq = diameter * diameter;


	for (size_t i = 0; i < m_state.particles.size() - 1; ++i)
	{
		auto first = m_state.particles[i];

		for (size_t j = i + 1; j < m_state.particles.size(); ++j)
		{
			auto second = m_state.particles[j];

			auto deltaPos = second.pos - first.pos;
			auto distSq = glm::dot(deltaPos, deltaPos);
			if (distSq > diameterSq)
				continue;

			auto dist = glm::sqrt(distSq);
			auto dir = deltaPos / dist;
			auto springLen = diameter - dist;

			const auto& vel1 = m_state.particles[i].vel;
			const auto& vel2 = m_state.particles[j].vel;

			auto force = SpringDamper(dir, vel1, vel2, springLen);

			if (i >= m_fixedCount)
				m_forces[i - m_fixedCount] -= force;
			if (j >= m_fixedCount)
				m_forces[j - m_fixedCount] += force;
		}

		if (i >= m_fixedCount)
			for (const auto& w : m_walls)
			{
				auto d = w.DistanceToLine(first.pos) - m_state.particleRad;
				if (d < 0.0f && glm::dot(w.normal(), first.pos) < 0.0f)
				{
					auto force = SpringDamper(w.normal(), first.vel, glm::vec2(0.0f), d);
					m_forces[i - m_fixedCount] -= force;
				}
			}
	}

	for (size_t i = m_fixedCount; i < m_state.particles.size(); ++i)
	{
		auto& p = m_state.particles[i];
		p.pos += p.vel * dt;
		
		auto& force = m_forces[i - m_fixedCount];
		force.y -= 0.5f;

		p.vel += force * dt;
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

	m_walls.emplace_back(topLeft, topRight);
	m_walls.emplace_back(topRight, bottomRight);
	m_walls.emplace_back(bottomRight, bottomLeft);
	m_walls.emplace_back(bottomLeft, topLeft);
}

void wing2d::simulation::cpu::CSimulationCpu::BuildWing()
{
	m_fixedCount = 0;

	std::vector<std::tuple<glm::vec2, glm::vec2>> pairs;
	for (size_t i = 0; i < m_state.airfoil.size() - 1; ++i)
		pairs.emplace_back(std::make_tuple(m_state.airfoil[i], m_state.airfoil[i + 1]));

	pairs.emplace_back(std::make_tuple(m_state.airfoil.back(), m_state.airfoil.front()));

	for (auto& t : pairs)
	{
		auto[first, second] = t;
		auto dir = second - first;
		auto len = glm::length(dir);
		dir /= len;

		size_t particles = size_t(glm::ceil(len / (m_state.particleRad)));

		for (size_t i = 0; i < particles; ++i)
		{
			auto nextPos = m_state.particleRad + i * (m_state.particleRad);

			m_state.particles[m_fixedCount++].pos = first + dir * nextPos;
			if (m_fixedCount >= m_state.particles.size())
				return;
		}
	}
}