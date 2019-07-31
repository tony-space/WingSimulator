#include "pch.hpp"

#include "CDerivativeSolver.hpp"
#include "CSimulationCpu.hpp"
#include "CBoundingBox.hpp"

using namespace wing2d::simulation::cpu;

static glm::vec2 SpringDamper(const glm::vec2& normal, const glm::vec2& vel1, const glm::vec2& vel2, float springLen)
{
	constexpr float stiffness = 10000.0f;
	constexpr float damp = 50.0f;
	auto v = glm::dot(vel1 - vel2, normal);
	auto force = normal * (springLen * stiffness + v * damp);
	return force;
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

	m_odeState = &state;
	m_forces.resize(particles);

	ResetForces();
	ParticleToParticle();
	ParticleToWing();
	ParticleToWall();
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

void CDerivativeSolver::ResetForces()
{
	std::fill(std::execution::par_unseq, m_forces.begin(), m_forces.end(), glm::vec2(0.0f));
}

void CDerivativeSolver::ParticleToParticle()
{
	const auto& state = m_simulation.GetState();
	const auto particles = state.particles;

	const auto* pos = m_odeState->data();
	const auto* vel = pos + particles;

	const auto rad = state.particleRad;
	const auto diameter = rad * 2.0f;

	m_p2pTree.Build(pos, particles, rad);

	m_potentialCollisionsList.resize(particles);

	std::for_each(std::execution::par_unseq, m_potentialCollisionsList.begin(), m_potentialCollisionsList.end(), [&](auto& collisionList)
	{
		collisionList.clear();
		size_t i = &collisionList - m_potentialCollisionsList.data();

		const auto &p1 = pos[i];
		const auto &v1 = vel[i];
		CBoundingBox box;
		box.AddPoint(glm::vec2(p1.x - rad, p1.y - rad));
		box.AddPoint(glm::vec2(p1.x + rad, p1.y + rad));

		m_p2pTree.Traverse(collisionList, box);

		for (size_t otherObjectIdx : collisionList)
		{
			if (otherObjectIdx == i)
				continue;

			const auto& p2 = pos[otherObjectIdx];
			const auto& v2 = vel[otherObjectIdx];

			auto force = ComputeForce(p1, v1, p2, v2, diameter);
			m_forces[i] -= force;
		}
	});
}

void CDerivativeSolver::ParticleToWing()
{
	const auto& state = m_simulation.GetState();
	const auto& wingParticles = m_simulation.GetWing();

	const auto particles = state.particles;
	auto pos = *m_odeState | boost::adaptors::sliced(0, particles);
	auto vel = *m_odeState | boost::adaptors::sliced(particles, particles * 2);
	const auto rad = state.particleRad;
	const auto diameter = rad * 2.0f;

	m_p2wTree.Build(wingParticles.data(), wingParticles.size(), rad);

	m_potentialCollisionsList.resize(particles);

	std::for_each(std::execution::par_unseq, m_potentialCollisionsList.begin(), m_potentialCollisionsList.end(), [&](auto& collisionList)
	{
		collisionList.clear();
		size_t i = &collisionList - m_potentialCollisionsList.data();

		const auto &p = pos[i];
		const auto &v = vel[i];
		CBoundingBox box;
		box.AddPoint(glm::vec2(p.x - rad, p.y - rad));
		box.AddPoint(glm::vec2(p.x + rad, p.y + rad));

		m_p2wTree.Traverse(collisionList, box);

		for (size_t wingIdx : collisionList)
		{
			const auto& wp = wingParticles[wingIdx];
			m_forces[i] -= ComputeForce(p, v, wp, glm::vec2(0.0f), diameter);
		}
	});
}

void CDerivativeSolver::ParticleToWall()
{
	const auto& state = m_simulation.GetState();
	const auto& walls = m_simulation.GetWalls();
	const auto particles = state.particles;

	auto pos = *m_odeState | boost::adaptors::sliced(0, particles);
	auto vel = *m_odeState | boost::adaptors::sliced(particles, particles * 2);
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