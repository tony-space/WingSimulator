#include "pch.hpp"
#include "CSimulationCpu.hpp"

using namespace wing2d::simulation;
using namespace wing2d::simulation::cpu;

glm::vec2 SpringDamper(const glm::vec2& normal, const glm::vec2& vel1, const glm::vec2& vel2, float springLen)
{
	constexpr float stiffness = 20000.0f;
	constexpr float damp = 50.0f;
	auto v = glm::dot(vel1 - vel2, normal);
	auto force = normal * (springLen * stiffness + v * damp);
	return force;
}

glm::vec4 getHeatMapColor(float value)
{
	value = fminf(fmaxf(value, 0.0f), 1.0f);

	static const size_t stages = 7;
	static const glm::vec3 heatMap[stages] = { {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f} };
	value *= stages - 1;
	int idx1 = int(value);
	int idx2 = idx1 + 1;
	float fract1 = value - float(idx1);
	return glm::vec4(heatMap[idx1] + fract1 * (heatMap[idx2] - heatMap[idx1]), 1.0f);
}

std::unique_ptr<ISimulation> wing2d::simulation::cpu::CreateSimulation()
{
	return std::make_unique<CSimulationCpu>();
}

void CSimulationCpu::ResetState(const SimulationState& state)
{
	if (!state.IsValid())
		throw std::runtime_error("state is invalid");

	m_state = state;
	m_walls.clear();
	BuildWalls();
	BuildWing();

	m_forces.resize(m_state.particles);
}

float CSimulationCpu::ComputeMinDeltaTime(float requestedDt) const
{
	auto timeRange = m_state.vel | boost::adaptors::transformed([&](const auto& v)
	{
		auto vel = glm::length(v);
		auto halfRadDt = m_state.particleRad * 0.5f / vel;
		return std::min(halfRadDt, requestedDt);
	});
	return std::reduce(timeRange.begin(), timeRange.end(), requestedDt, [](auto t1, auto t2) {return t1 < t2 ? t1 : t2; });
}

void CSimulationCpu::ResetForces()
{
	std::fill(m_forces.begin(), m_forces.end(), glm::vec2(0.0f));
}

glm::vec2 CSimulationCpu::ComputeForce(const glm::vec2& pos1, const glm::vec2& vel1, const glm::vec2& pos2, const glm::vec2& vel2, float diameter)
{
	auto deltaPos = pos2 - pos1;
	auto distSq = glm::dot(deltaPos, deltaPos);
	if (distSq > (diameter * diameter))
		return glm::vec2(0.0f);

	auto dist = glm::sqrt(distSq);
	auto dir = deltaPos / dist;
	auto springLen = diameter - dist;

	auto force = SpringDamper(dir, vel1, vel2, springLen);
	return force;
}

void CSimulationCpu::ParticleToParticle()
{
	const auto diameter = m_state.particleRad * 2.0f;

	for (size_t i = 0; i < m_state.particles - 1; ++i)
	{
		const auto &p1 = m_state.pos[i];
		const auto &v1 = m_state.vel[i];

		for (size_t j = i + 1; j < m_state.particles; ++j)
		{
			const auto& p2 = m_state.pos[j];
			const auto& v2 = m_state.vel[j];
			auto force = ComputeForce(p1, v1, p2, v2, diameter);
			m_forces[i] -= force;
			m_forces[j] += force;
		}
	}
}

void CSimulationCpu::ParticleToWing()
{
	const auto diameter = m_state.particleRad * 2.0f;

	for (size_t i = 0; i < m_state.particles; ++i)
	{
		const auto &p = m_state.pos[i];
		const auto &v = m_state.vel[i];

		for (const auto& wp : m_wingParticles)
		{
			m_forces[i] -= ComputeForce(p, v, wp, glm::vec2(0.0f), diameter);
		}
	}
}

void CSimulationCpu::ParticleToWall()
{
	for (size_t i = 0; i < m_state.particles; ++i)
	{
		const auto &p = m_state.pos[i];
		const auto &v = m_state.vel[i];

		for (const auto& w : m_walls)
		{
			auto d = w.DistanceToLine(p) - m_state.particleRad;
			if (d < 0.0f && glm::dot(w.normal(), p) < 0.0f)
			{
				auto force = SpringDamper(w.normal(), v, glm::vec2(0.0f), d);
				m_forces[i] -= force;
			}
		}
	}
}

void CSimulationCpu::MoveParticles(float dt)
{
	for (size_t i = 0; i < m_state.particles; ++i)
	{
		auto &p = m_state.pos[i];
		auto &v = m_state.vel[i];
		auto &f = m_forces[i];

		f.y -= 0.5f;
		p += v * dt;
		v += f * dt;
	}
}

void CSimulationCpu::ColorParticles()
{
	for (size_t i = 0; i < m_state.particles; ++i)
	{
		const auto& force = m_forces[i];
		m_state.color[i] = getHeatMapColor(glm::log(glm::length(force) + 1.0f) / 8.0f + 0.15f);
	}
}

float CSimulationCpu::Update(float dt)
{
	dt = ComputeMinDeltaTime(dt);

	ResetForces();
	ParticleToParticle();
	ParticleToWing();
	ParticleToWall();
	MoveParticles(dt);
	ColorParticles();
	return dt;
}

const SimulationState& CSimulationCpu::GetState() const
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

void CSimulationCpu::BuildWing()
{
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
			m_wingParticles.emplace_back(first + dir * nextPos);
		}
	}
}
