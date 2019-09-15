#include "pch.hpp"

#include "CDerivativeSolver.hpp"
#include "CSimulationCpu.hpp"
#include "CBoundingBox.hpp"

using namespace wing2d::simulation;
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

void CDerivativeSolver::Derive(const OdeState_t& prevState, const OdeState_t& curState, OdeState_t& outDerivative)
{
	const auto particles = m_simulation.GetState().particles;

	m_odeState = &curState;
	m_forces.resize(particles);

	ResetForces();
	BuildTree();
	ResolveCollisions();
	ParticleToWall();
	ApplyGravity();

	std::copy(std::execution::par_unseq, curState.cbegin() + particles, curState.cend(), outDerivative.begin());
	std::copy(std::execution::par_unseq, m_forces.cbegin(), m_forces.cend(), outDerivative.begin() + particles);
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

	m_allObjects.resize(gasParticles + wingParticles);
	const auto* pos = m_odeState->data();
	std::for_each(std::execution::par_unseq, m_allObjects.begin(), m_allObjects.end(), [&](CBoundingBox& objBox)
	{
		size_t i = &objBox - m_allObjects.data();
		auto min = -glm::vec2(rad, rad);
		auto max = glm::vec2(rad, rad);
		auto p = i < gasParticles ? pos[i] : wing[i - gasParticles];

		objBox.SetMin(min + p);
		objBox.SetMax(max + p);
	});

	m_particlesTree.Build(m_allObjects);
}

void CDerivativeSolver::ResolveCollisions()
{
	const auto& state = m_simulation.GetState();
	const auto& wing = m_simulation.GetWing();
	const auto gasParticles = state.particles;

	const auto* pos = m_odeState->data();
	const auto* vel = pos + gasParticles;
	const auto diameter = state.particleRad * 2.0f;

	const auto corner = glm::vec2(state.particleRad, state.particleRad);

	m_potentialCollisionsList.resize(gasParticles);

	std::for_each(std::execution::par_unseq, m_potentialCollisionsList.begin(), m_potentialCollisionsList.end(), [&](auto& collisionList)
	{
		size_t i = &collisionList - m_potentialCollisionsList.data();

		const auto& p1 = pos[i];
		const auto& v1 = vel[i];
		CBoundingBox box(p1 - corner, p1 + corner);

		m_particlesTree.Traverse(box, collisionList);

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

		const auto& p = pos[i];
		const auto& v = vel[i];

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