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
	m_combinedForces.resize(particles);
	m_p2pForces.resize(particles);
	m_p2wForces.resize(particles);

	ResetForces();
	ParticleToParticle();
	ParticleToWing();
	ParticleToWall();
	CombineForces();

	std::copy(state.cbegin() + particles, state.cend(), derivative.begin());
	std::copy(m_combinedForces.cbegin(), m_combinedForces.cend(), derivative.begin() + particles);
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
	std::fill(std::execution::par_unseq, m_p2pForces.begin(), m_p2pForces.end(), glm::vec2(0.0f));
	std::fill(std::execution::par_unseq, m_p2wForces.begin(), m_p2wForces.end(), glm::vec2(0.0f));
	std::fill(std::execution::par_unseq, m_combinedForces.begin(), m_combinedForces.end(), glm::vec2(0.0f));
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

	m_potentialCollisionsListP2p.resize(particles);

	std::for_each(std::execution::par_unseq, m_potentialCollisionsListP2p.begin(), m_potentialCollisionsListP2p.end(), [&](auto& collisionList)
	{
		collisionList.clear();
		size_t i = &collisionList - m_potentialCollisionsListP2p.data();

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
			m_p2pForces[i] -= force;
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

	m_potentialCollisionsListP2w.resize(particles);

	std::for_each(std::execution::par_unseq, m_potentialCollisionsListP2w.begin(), m_potentialCollisionsListP2w.end(), [&](auto& collisionList)
	{
		collisionList.clear();
		size_t i = &collisionList - m_potentialCollisionsListP2w.data();

		const auto &p = pos[i];
		const auto &v = vel[i];
		CBoundingBox box;
		box.AddPoint(glm::vec2(p.x - rad, p.y - rad));
		box.AddPoint(glm::vec2(p.x + rad, p.y + rad));

		m_p2wTree.Traverse(collisionList, box);

		for (size_t wingIdx : collisionList)
		{
			const auto& wp = wingParticles[wingIdx];
			m_p2wForces[i] -= ComputeForce(p, v, wp, glm::vec2(0.0f), diameter);
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
	std::for_each(std::execution::par_unseq, m_combinedForces.begin(), m_combinedForces.end(), [&](auto& force)
	{
		size_t i = &force - m_combinedForces.data();

		const auto &p = pos[i];
		const auto &v = vel[i];

		for (const auto& w : walls)
		{
			auto d = w.DistanceToLine(p) - state.particleRad;
			if (d < 0.0f && glm::dot(w.normal(), p) < 0.0f)
			{
				auto f = SpringDamper(w.normal(), v, glm::vec2(0.0f), d);
				force -= f;
			}
		}
	});
}

void CDerivativeSolver::CombineForces()
{
	std::transform(std::execution::par_unseq, m_combinedForces.cbegin(), m_combinedForces.cend(), m_combinedForces.begin(), [&](auto& force)
	{
		size_t i = &force - m_combinedForces.data();

		auto f = force + m_p2pForces[i] + m_p2wForces[i];
		//f.y -= 0.5f;
		f.x += 0.5f;

		return f;
	});
}