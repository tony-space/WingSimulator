#include "pch.hpp"
#include "CSimulationCpu.hpp"

using namespace wing2d::simulation;
using namespace wing2d::simulation::serialization;
using namespace wing2d::simulation::cpu;

std::unique_ptr<ISimulation> wing2d::simulation::cpu::CreateSimulation()
{
	return std::make_unique<CSimulationCpu>();
}

CSimulationCpu::SLine::SLine(const glm::vec2& v1, const glm::vec2& v2)
{
	auto dt = v2 - v1;

	origin = v1;
	length = glm::length(dt);
	if (length < 1e-16f)
		length = 0.0f;

	ray = length > 0.0f ? dt / length : glm::vec2(0.0f);
	normal = glm::vec2(ray.y, -ray.x);
}


bool CSimulationCpu::SLine::Intersected(const glm::vec2& spherePos, float sphereRad, glm::vec2& contactNorm, float& outDepth)
{
	//auto toSphereCenter = spherePos - origin;

	//if (length == 0.0f)
	//{
	//	auto distanceToSphere = glm::length(toSphereCenter);
	//	//if(distanceToSphere)
	//}

	//auto spherePosProj = glm::dot(C, ray);

	return false;
}

void CSimulationCpu::ResetState(const SimulationState& state)
{
	m_state = state;

	m_lines.resize(0);
	m_lines.reserve(state.wing.size());

}

float CSimulationCpu::Update(float dt)
{
	float left = -m_state.worldSize.width * 0.5f;
	float right = m_state.worldSize.width * 0.5f;

	float bottom = -m_state.worldSize.height * 0.5f;
	float top = m_state.worldSize.height * 0.5f;
	
	for (auto& p : m_state.particles)
	{
		p.pos += p.vel * dt;
		if (p.pos.x + m_state.particleRad > right)
		{
			p.pos.x = right - m_state.particleRad;
			p.vel.x = -glm::abs(p.vel.x);
		}

		if (p.pos.x - m_state.particleRad < left)
		{
			p.pos.x = left + m_state.particleRad;
			p.vel.x = glm::abs(p.vel.x);
		}

		if (p.pos.y + m_state.particleRad > top)
		{
			p.pos.y = top - m_state.particleRad;
			p.vel.y = -glm::abs(p.vel.y);
		}

		if (p.pos.y - m_state.particleRad < bottom)
		{
			p.pos.y = bottom + m_state.particleRad;
			p.vel.y = glm::abs(p.vel.y);
		}
	}

	return dt;
}

void CSimulationCpu::GetState(SimulationState& outState)
{
	outState = m_state;
}
