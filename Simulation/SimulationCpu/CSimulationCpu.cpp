#include "pch.hpp"
#include "CSimulationCpu.hpp"

using namespace wing2d::simulation;
using namespace wing2d::simulation::cpu;

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

CSimulationCpu::CSimulationCpu() : m_derivativeSolver(*this)
{

}

void CSimulationCpu::ResetState(const SimulationState& state)
{
	if (!state.IsValid())
		throw std::runtime_error("state is invalid");

	m_state = state;
	BuildWalls();
	BuildWing();

	m_odeState.resize(m_state.particles * 2);
	m_odeNextState.resize(m_state.particles * 2);
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

void CSimulationCpu::ColorParticles(float dt)
{
	auto forces = boost::combine(m_odeState, m_odeNextState) | boost::adaptors::sliced(m_state.particles, m_state.particles * 2);
	auto colors = forces | boost::adaptors::transformed([&](const auto& tuple)
	{
		glm::vec2 vel1;
		glm::vec2 vel2;
		boost::tie(vel1, vel2) = tuple;
		auto force = (vel2 - vel1) / dt;
		return getHeatMapColor(glm::log(glm::length(force) + 1.0f) / 10.0f + 0.15f);
	});

	boost::range::copy_n(colors, m_state.particles, m_state.color.begin());
}

float CSimulationCpu::Update(float dt)
{
	dt = ComputeMinDeltaTime(dt);

	auto pos1 = m_odeState | boost::adaptors::sliced(0, m_state.particles);
	auto vel1 = m_odeState | boost::adaptors::sliced(m_state.particles, m_state.particles * 2);
	boost::range::copy_n(m_state.pos, m_state.particles, pos1.begin());
	boost::range::copy_n(m_state.vel, m_state.particles, vel1.begin());

	m_odeSolver.RungeKutta(m_odeState, m_derivativeSolver, dt, m_odeNextState);

	auto pos2 = m_odeNextState | boost::adaptors::sliced(0, m_state.particles);
	auto vel2 = m_odeNextState | boost::adaptors::sliced(m_state.particles, m_state.particles * 2);
	boost::range::copy_n(pos2, m_state.particles, m_state.pos.begin());
	boost::range::copy_n(vel2, m_state.particles, m_state.vel.begin());

	ColorParticles(dt);

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

	m_walls.clear();
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
