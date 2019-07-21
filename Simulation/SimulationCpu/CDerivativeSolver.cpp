#include "pch.hpp"

#include "CDerivativeSolver.hpp"
#include "CSimulationCpu.hpp"

using namespace wing2d::simulation::cpu;

static glm::vec2 SpringDamper(const glm::vec2& normal, const glm::vec2& vel1, const glm::vec2& vel2, float springLen)
{
	constexpr float stiffness = 20000.0f;
	constexpr float damp = 50.0f;
	auto v = glm::dot(vel1 - vel2, normal);
	auto force = normal * (springLen * stiffness + v * damp);
	return force;
}
static inline uint64_t ExpandBy1(uint32_t val)
{
	uint64_t result = val;

	result = ((result << 16) | result) & 0x0000FFFF0000FFFF;
	result = ((result << 8) | result) & 0x00FF00FF00FF00FF;
	result = ((result << 4) | result) & 0x0F0F0F0F0F0F0F0F;
	result = ((result << 2) | result) & 0x3333333333333333;
	result = ((result << 1) | result) & 0x5555555555555555;

	return result;
}

static inline uint64_t Morton2D(uint32_t x, uint32_t y)
{
	return (ExpandBy1(x) << 1) | ExpandBy1(y);
}

CDerivativeSolver::CDerivativeSolver(const CSimulationCpu& sim) :
	m_simulation(sim)
{
}

void CDerivativeSolver::operator()(const OdeState_t& state, OdeState_t& derivative)
{
	const auto particles = m_simulation.GetState().particles;
	if (state.size() != particles * 2 || derivative.size() != particles * 2)
		throw std::runtime_error("incorrect state/derivative size");

	m_forces.resize(particles);

	ResetForces();
	BuildTree(state);
	ParticleToParticle(state);
	ParticleToWing(state);
	ParticleToWall(state);
	ApplyGravity();

	std::copy(state.cbegin() + particles, state.cend(), derivative.begin());
	std::copy(m_forces.cbegin(), m_forces.cend(), derivative.begin() + particles);
}

glm::vec2 CDerivativeSolver::ComputeForce(const glm::vec2& pos1, const glm::vec2& vel1, const glm::vec2& pos2, const glm::vec2& vel2, float diameter)
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

void CDerivativeSolver::BuildTree(const OdeState_t& odeState)
{
	m_particlesBox = CBoundingBox();

	const auto& state = m_simulation.GetState();
	const auto particles = state.particles;

	auto pos = odeState | boost::adaptors::sliced(0, particles);
	m_particlesBox.AddPoints(pos.begin(), pos.end());
	m_sortedMortonCodes.resize(particles);

	auto boxSize = m_particlesBox.max() - m_particlesBox.min();

	for(size_t i = 0; i < particles; ++i)
	{
		const auto& p = pos[i];
		auto normalized = (p - m_particlesBox.min()) / boxSize;
		assert(normalized.x >= 0.0f);
		assert(normalized.y >= 0.0f);
		assert(normalized.x <= 1.0f);
		assert(normalized.y <= 1.0f);

		uint32_t x = uint32_t(glm::round(normalized.x * double(0xFFFFFFFF)));
		uint32_t y = uint32_t(glm::round(normalized.y * double(0xFFFFFFFF)));
		uint64_t morton = Morton2D(x, y);

		m_sortedMortonCodes[i] = std::make_tuple(morton, i);
	}

}

void CDerivativeSolver::ResetForces()
{
	std::fill(m_forces.begin(), m_forces.end(), glm::vec2(0.0f));
}

void CDerivativeSolver::ParticleToParticle(const OdeState_t& odeState)
{
	const auto& state = m_simulation.GetState();
	const auto particles = state.particles;

	auto pos = odeState | boost::adaptors::sliced(0, particles);
	auto vel = odeState | boost::adaptors::sliced(particles, particles * 2);
	const auto diameter = state.particleRad * 2.0f;

	for (size_t i = 0; i < particles - 1; ++i)
	{
		const auto& p1 = pos[i];
		const auto& v1 = vel[i];

		for (size_t j = i + 1; j < state.particles; ++j)
		{
			const auto& p2 = pos[j];
			const auto& v2 = vel[j];
			auto force = ComputeForce(p1, v1, p2, v2, diameter);
			m_forces[i] -= force;
			m_forces[j] += force;
		}
	}
}

void CDerivativeSolver::ParticleToWing(const OdeState_t& odeState)
{
	const auto& state = m_simulation.GetState();
	const auto& wingParticles = m_simulation.GetWing();
	const auto particles = state.particles;

	auto pos = odeState | boost::adaptors::sliced(0, particles);
	auto vel = odeState | boost::adaptors::sliced(particles, particles * 2);
	const auto diameter = state.particleRad * 2.0f;

	for (size_t i = 0; i < particles; ++i)
	{
		const auto &p = pos[i];
		const auto &v = vel[i];

		for (const auto& wp : wingParticles)
		{
			m_forces[i] -= ComputeForce(p, v, wp, glm::vec2(0.0f), diameter);
		}
	}
}

void CDerivativeSolver::ParticleToWall(const OdeState_t& odeState)
{
	const auto& state = m_simulation.GetState();
	const auto& walls = m_simulation.GetWalls();
	const auto particles = state.particles;

	auto pos = odeState | boost::adaptors::sliced(0, particles);
	auto vel = odeState | boost::adaptors::sliced(particles, particles * 2);
	for (size_t i = 0; i < particles; ++i)
	{
		const auto &p = pos[i];
		const auto &v = vel[i];

		for (const auto& w : walls)
		{
			auto d = w.DistanceToLine(p) - state.particleRad;
			if (d < 0.0f && glm::dot(w.normal(), p) < 0.0f)
			{
				auto force = SpringDamper(w.normal(), v, glm::vec2(0.0f), d);
				m_forces[i] -= force;
			}
		}
	}
}

void CDerivativeSolver::ApplyGravity()
{
	for (auto& f : m_forces)
		f.y -= 0.5f;
}
