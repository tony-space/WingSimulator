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
	return -normal * (springLen * stiffness + v * damp);
}

static glm::vec2 SpringDamper2(const glm::vec2& normal, const glm::vec2& vel1, const glm::vec2& vel2, float springLen)
{
	constexpr float stiffness = 50000.0f;
	constexpr float damp = 50.0f;
	auto v = glm::dot(vel1 - vel2, normal);
	return -normal * (springLen * stiffness + v * damp);
}

CDerivativeSolver::CDerivativeSolver(const CSimulationCpu& sim) :
	m_particles(sim.GetParticlesCount()),
	m_particleRadius(sim.GetParticleRadius()),
	m_wing(sim.GetWing()),
	m_walls(sim.GetWalls())
{
	m_forces.resize(m_particles);
	m_potentialCollisionsList.resize(m_particles);
	m_particlesBoundingBoxes.resize(m_particles);

	std::vector<CBoundingBox> wingBoxes(m_wing.size());

	std::transform(std::execution::par_unseq, m_wing.cbegin(), m_wing.cend(), wingBoxes.begin(), [](const CLineSegment& segment)
	{
		return CBoundingBox(segment.First(), segment.Second());
	});

	m_wingTree.Build(wingBoxes);
}

void CDerivativeSolver::Derive(const OdeState_t& curState, OdeState_t& outDerivative)
{
	ResetForces();
	BuildParticlesTree(curState);
	ResolveParticleParticleCollisions(curState);
	ResolveParticleWingCollisions(curState);
	ParticleToWall(curState);
	ApplyGravity();
	BuildDerivative(curState, outDerivative);
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

void CDerivativeSolver::BuildParticlesTree(const OdeState_t& curState)
{
	auto posBegin = curState.cbegin();
	auto posEnd = curState.cbegin() + m_particles;

	auto corner = glm::vec2(m_particleRadius);

	std::transform(std::execution::par_unseq, posBegin, posEnd, m_particlesBoundingBoxes.begin(), [&](const glm::vec2& p)
	{
		auto min = p - corner;
		auto max = p + corner;

		return CBoundingBox(min, max);
	});

	m_particlesTree.Build(m_particlesBoundingBoxes);
}

void CDerivativeSolver::ResolveParticleParticleCollisions(const OdeState_t& curState)
{
	const auto pos = curState.cbegin();
	const auto vel = pos + m_particles;
	const auto corner = glm::vec2(m_particleRadius);
	const auto diameter = m_particleRadius * 2.0f;

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

			const auto& p2 = pos[otherObjectIdx];
			const auto& v2 = vel[otherObjectIdx];

			force += ComputeForce(p1, v1, p2, v2, diameter);
		}

		m_forces[i] = force;
	});
}

void CDerivativeSolver::ResolveParticleWingCollisions(const OdeState_t& curState)
{
	typedef std::pair<const CLineSegment*, float> segmentInfo_t;
	const auto& velBegin = curState.cbegin() + m_particles;

	std::for_each_n(std::execution::par_unseq, curState.cbegin(), m_particles, [&](const glm::vec2& pos)
	{
		size_t i = &pos - curState.data();

		const auto area = glm::vec2(m_particleRadius * 4.0f);
		const auto particleCollisionBox = CBoundingBox(pos - area, pos + area);
		auto & potentialCollisions = m_potentialCollisionsList[i];
		m_wingTree.Traverse(particleCollisionBox, potentialCollisions);

		if (potentialCollisions.empty())
			return;

		auto closestSegmentInfo = std::transform_reduce(std::execution::par_unseq, potentialCollisions.cbegin(), potentialCollisions.cend(), segmentInfo_t(nullptr, INFINITY), [&](const segmentInfo_t& p1, const segmentInfo_t& p2)
		{
			if (p1.second < p2.second)
			{
				return p1;
			}
			else if (p1.second == p2.second)
			{
				auto delta1 = pos - p1.first->First();
				auto delta2 = pos - p2.first->First();

				auto dot1 = glm::dot(delta1, p1.first->Normal());
				auto dot2 = glm::dot(delta2, p2.first->Normal());

				return dot1 > dot2 ? p1 : p2;
			}
			else
			{
				return p2;
			}
		}, [&](const size_t wingSegmentIdx)
		{
			const auto& wingSegment = m_wing[wingSegmentIdx];
			return std::make_pair(&wingSegment, glm::distance(wingSegment.ClosestPoint(pos), pos));
		});

		const auto& closestSegment = *closestSegmentInfo.first;
		const auto height = glm::dot(closestSegment.Normal(), pos - closestSegment.First());

		if (height <= m_particleRadius)
		{
			const auto penetration = height - m_particleRadius;
			m_forces[i] += SpringDamper2(closestSegment.Normal(), velBegin[i], glm::vec2(0.0f), penetration);
		}
	});
}

void CDerivativeSolver::ParticleToWall(const OdeState_t& curState)
{
	auto pos = curState.cbegin();
	auto vel = pos + m_particles;
	std::for_each(std::execution::par_unseq, m_forces.begin(), m_forces.end(), [&](auto& force)
	{
		size_t i = &force - m_forces.data();

		const auto& p = pos[i];
		const auto& v = vel[i];

		for (const auto& w : m_walls)
		{
			auto d = w.DistanceToLine(p) - m_particleRadius;
			if (d < 0.0f && glm::dot(w.Normal(), p) < 0.0f)
			{
				force += SpringDamper(w.Normal(), v, glm::vec2(0.0f), d);
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

void CDerivativeSolver::BuildDerivative(const OdeState_t& curState, OdeState_t& outDerivative) const
{
	std::copy(std::execution::par_unseq, curState.cbegin() + m_particles, curState.cend(), outDerivative.begin());
	std::copy(std::execution::par_unseq, m_forces.cbegin(), m_forces.cend(), outDerivative.begin() + m_particles);
}