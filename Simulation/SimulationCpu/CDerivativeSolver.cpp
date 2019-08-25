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
	BuildTree();
	ResolveCollisions();
	ParticleToWall();
	ApplyGravity();

	std::copy(std::execution::par_unseq, state.cbegin() + particles, state.cend(), derivative.begin());
	std::copy(std::execution::par_unseq, m_forces.cbegin(), m_forces.cend(), derivative.begin() + particles);
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

void CDerivativeSolver::BuildTree()
{
	const auto& state = m_simulation.GetState();
	const auto gasParticles = state.particles;
	
	const auto& wing = m_simulation.GetWing();
	const auto wingParticles = wing.size();
	
	const auto rad = state.particleRad;

	m_allParticles.resize(gasParticles + wingParticles);
	const auto* pos = m_odeState->data();
	std::for_each(std::execution::par_unseq, m_allParticles.begin(), m_allParticles.end(), [&](auto& vecPtr)
	{
		size_t i = &vecPtr - m_allParticles.data();
		if (i < gasParticles)
			vecPtr = pos[i];
		else
			vecPtr = wing[i - gasParticles];
	});

	m_particlesTree.Build(m_allParticles, rad);
}

void CDerivativeSolver::ResolveCollisions()
{
	const auto& state = m_simulation.GetState();
	const auto& wing = m_simulation.GetWing();
	const auto gasParticles = state.particles;

	const auto* pos = m_odeState->data();
	const auto* vel = pos + gasParticles;
	const auto rad = state.particleRad;
	const auto diameter = rad * 2.0f;
	
	m_potentialCollisionsList.resize(gasParticles);

	std::for_each(std::execution::par_unseq, m_potentialCollisionsList.begin(), m_potentialCollisionsList.end(), [&](auto& collisionList)
	{
		collisionList.clear();
		size_t i = &collisionList - m_potentialCollisionsList.data();

		const auto &p1 = pos[i];
		const auto &v1 = vel[i];
		CBoundingBox box;
		box.AddPoint(glm::vec2(p1.x - rad, p1.y - rad));
		box.AddPoint(glm::vec2(p1.x + rad, p1.y + rad));

		m_particlesTree.Traverse(collisionList, box);

		glm::vec2 force(0.0f);

		for (size_t otherObjectIdx : collisionList)
		{
			if (otherObjectIdx == i)
				continue;

			if (otherObjectIdx < gasParticles)
			{
				const auto& p2 = pos[otherObjectIdx];
				const auto& v2 = vel[otherObjectIdx];

				force -= ComputeForce(p1, v1, p2, v2, diameter);
			}
			else
			{
				const auto& p2 = wing[otherObjectIdx - gasParticles];
				force -= ComputeForce(p1, v1, p2, glm::vec2(0.0f), diameter);
			}
		}

		m_forces[i] = force;
	});
}

void CDerivativeSolver::ParticleToWall()
{
	const auto& state = m_simulation.GetState();
	const auto& walls = m_simulation.GetWalls();
	const auto particles = state.particles;

	auto pos = m_odeState->begin();
	auto vel = pos + particles;
	std::for_each(std::execution::par_unseq, m_forces.begin(), m_forces.end(), [&](auto& force)
	{
		size_t i = &force - m_forces.data();

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

void CDerivativeSolver::ApplyGravity()
{
	std::transform(std::execution::par_unseq, m_forces.cbegin(), m_forces.cend(), m_forces.begin(), [&](auto force)
	{
		force.x += 0.5f;
		return force;
	});
}