#include "pch.hpp"
#include "CSimulationCpu.hpp"
#include "CBoundingBox.hpp"

using namespace wing2d::simulation;
using namespace wing2d::simulation::serialization;
using namespace wing2d::simulation::cpu;

struct CollisionResponse
{
	glm::vec2 contactPoint;
	glm::vec2 normal;
	float timeToCollision;
};

bool PredictCollisionTime(const glm::vec2& pos, const glm::vec2& vel, float rad, const glm::vec2& point, CollisionResponse& out)
{
	//C0 + Vt = C1
	//(P - C1) dot (P - C1) = r^2

	auto deltaPos = point - pos;
	auto a = glm::dot(vel, vel);
	auto b = 2.0f * glm::dot(vel, deltaPos);
	auto c = glm::dot(deltaPos, deltaPos) - rad * rad;

	if (glm::abs(a) < 1e-16)
		return false;

	if (b <= 0.0f)
		return false;

	auto D = b * b - 4 * a * c;
	if (D < 0)
		return false;
	auto sqrtD = glm::sqrt(D);
	auto t1 = (-b - sqrtD) / (2.0f * a);
	auto t2 = (-b + sqrtD) / (2.0f * a);

	auto t = INFINITY;
	if (t1 > 0.0f)
		t = std::min(t, t1);

	if (t2 > 0.0f)
		t = std::min(t, t2);

	if (t == INFINITY)
		return false;

	out.timeToCollision = t;
	out.contactPoint = point;
	out.normal = glm::normalize(pos - point);

	return true;
}

std::unique_ptr<ISimulation> wing2d::simulation::cpu::CreateSimulation()
{
	return std::make_unique<CSimulationCpu>();
}

void CSimulationCpu::ResetState(const SimulationState& state)
{
	m_state = state;
	BuildWalls();
}

float CSimulationCpu::Update(float dt)
{
	CollisionResponse closestCollision;
	closestCollision.timeToCollision = INFINITY;
	size_t particleId = -1;

	for (size_t i = 0; i < m_state.particles.size(); ++i)
	{
		const auto& particle = m_state.particles[i];

		for (const auto& point : m_state.airfoil)
		{
			CollisionResponse response;
			if (PredictCollisionTime(particle.pos, particle.vel, m_state.particleRad, point, response))
			{
				if (response.timeToCollision < closestCollision.timeToCollision)
				{
					closestCollision = response;
					particleId = i;
				}
			}
		}
	}

	dt = std::min(dt, closestCollision.timeToCollision);

	for (auto& p : m_state.particles)
	{
		p.pos += p.vel * dt;

		for (const auto& w : m_walls)
		{
			auto distance = w.DistanceToLine(p.pos) - m_state.particleRad;
			if (distance <= 0.0f && glm::dot(p.vel, w.normal()) < 0.0f)
				p.vel = glm::reflect(p.vel, w.normal());
		}
	}

	if (particleId != -1)
	{
		auto& p = m_state.particles[particleId];
		p.vel = glm::reflect(p.vel, closestCollision.normal);
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

	m_walls.clear();
	m_walls.reserve(4);
	m_walls.emplace_back(topLeft, topRight);
	m_walls.emplace_back(topRight, bottomRight);
	m_walls.emplace_back(bottomRight, bottomLeft);
	m_walls.emplace_back(bottomLeft, topLeft);
}