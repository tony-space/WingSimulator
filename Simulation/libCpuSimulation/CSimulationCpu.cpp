#include "pch.hpp"
#include "CSimulationCpu.hpp"

using namespace wing2d::simulation;
using namespace wing2d::simulation::cpu;

static SimulationState::vec4 getHeatMapColor(float value)
{
	value = fminf(fmaxf(value, 0.0f), 1.0f);

	static const size_t stages = 7;
	static const glm::vec3 heatMap[stages] = { {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f} };
	value *= stages - 1;
	int idx1 = int(value);
	int idx2 = idx1 + 1;
	float fract1 = value - float(idx1);
	auto result = heatMap[idx1] + fract1 * (heatMap[idx2] - heatMap[idx1]);

	return std::make_tuple(result.x, result.y, result.z, 1.0f);
}

static SimulationState::vec2 VecToTuple(const glm::vec2& v)
{
	return std::make_tuple(v.x, v.y);
};

static glm::vec2 TupleToVec(const SimulationState::vec2& v)
{
	auto& [x, y] = v;
	return glm::vec2(x, y);
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
	BuildWalls();
	BuildWing();

	auto size = m_state.particles * 2;

	m_curOdeState.resize(size);
	m_nextOdeState.resize(size);

	auto pos = m_curOdeState.begin();
	auto vel = pos + m_state.particles;

	std::transform(std::execution::par_unseq, m_state.pos.cbegin(), m_state.pos.cend(), pos, TupleToVec);
	std::transform(std::execution::par_unseq, m_state.vel.cbegin(), m_state.vel.cend(), vel, TupleToVec);

	m_odeSolver = std::make_unique<CForwardEulerSolver>(std::make_unique<CDerivativeSolver>(*this));
}

float CSimulationCpu::ComputeMinDeltaTime(float requestedDt) const
{
	auto rad = m_state.particleRad;
	auto velBegin = m_curOdeState.cbegin() + m_state.particles;
	auto velEnd = m_curOdeState.cend();

	return std::transform_reduce(std::execution::par_unseq, velBegin, velEnd, requestedDt, [](const auto t1, const auto t2)
	{
		return std::min(t1, t2);
	}, [&](const auto& v)
	{
		auto vel = glm::length(v);
		auto radDt = rad / vel;
		return radDt;
	});
}

float CSimulationCpu::Update(float dt)
{
	dt = ComputeMinDeltaTime(dt);

	m_odeSolver->NextState(m_curOdeState, dt, m_nextOdeState);

	ColorParticles(dt);

	m_nextOdeState.swap(m_curOdeState);

	return dt;
}

void CSimulationCpu::ColorParticles(float dt)
{
	auto curVelBegin = m_curOdeState.cbegin() + m_state.particles;
	auto curVelEnd = curVelBegin + m_state.particles;
	auto nextVelBegin = m_nextOdeState.cbegin() + m_state.particles;
	std::transform(std::execution::par_unseq, curVelBegin, curVelEnd, nextVelBegin, m_state.color.begin(), [&](const auto& vel1, const auto& vel2)
	{
		auto force = (vel2 - vel1) / dt;
		return getHeatMapColor(glm::log(glm::length(force) + 1.0f) / 10.0f + 0.15f);
	});
}

const SimulationState& CSimulationCpu::GetState()
{
	auto pos = m_curOdeState.cbegin();
	auto vel = pos + m_state.particles;
	auto end = m_curOdeState.cend();

	std::transform(std::execution::par_unseq, pos, vel, m_state.pos.begin(), VecToTuple);
	std::transform(std::execution::par_unseq, vel, end, m_state.vel.begin(), VecToTuple);

	return m_state;
}

size_t CSimulationCpu::GetParticlesCount() const
{
	return m_state.particles;
}

float CSimulationCpu::GetParticleRadius() const
{
	return m_state.particleRad;
}

const std::vector<CLineSegment>& CSimulationCpu::GetWalls() const
{
	return m_walls;
}

const std::vector<CLineSegment>& CSimulationCpu::GetWing() const
{
	return m_wing;
}

void CSimulationCpu::BuildWalls()
{
	glm::vec2 corner(m_state.worldSize.width / 2.0f, m_state.worldSize.height / 2.0f);
	glm::vec2 topLeft(-corner.x, corner.y);
	glm::vec2 topRight(corner.x, corner.y);
	glm::vec2 bottomRight(corner.x, -corner.y);
	glm::vec2 bottomLeft(-corner.x, -corner.y);

	m_walls.clear();
	m_walls.emplace_back(topRight, topLeft);
	m_walls.emplace_back(bottomLeft, bottomRight);
}

void CSimulationCpu::BuildWing()
{
	m_wing.clear();

	auto createSegment = [](const SimulationState::vec2& a, const SimulationState::vec2& b)
	{
		auto& [x1, y1] = a;
		auto& [x2, y2] = b;
		return CLineSegment(glm::vec2(x1, y1), glm::vec2(x2, y2));
	};

	for (size_t i = 0; i < m_state.airfoil.size() - 1; ++i)
		m_wing.emplace_back(createSegment(m_state.airfoil[i], m_state.airfoil[i + 1]));
	m_wing.emplace_back(createSegment(m_state.airfoil.back(), m_state.airfoil.front()));
}
