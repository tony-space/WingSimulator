#include "pch.hpp"
#include "CLineSegment.hpp"

using namespace wing2d::simulation::cpu;

CLineSegment::CLineSegment(const glm::vec2& a, const glm::vec2& b)
{
	auto delta = b - a;
	m_length = glm::length(delta);
	m_first = a;
	m_second = b;

	if (m_length > 1e-16f)
		m_ray = delta / m_length;
	else
		m_length = 0.0f;

	m_normal = glm::vec2(m_ray.y, -m_ray.x);
	m_distance = glm::dot(a, m_normal);
}

bool wing2d::simulation::cpu::CLineSegment::PredictCollision(const glm::vec2& pos, const glm::vec2& vel, float rad, collisions::SCollisionForecast& out) const
{
	out.timeToCollision = INFINITY;

	collisions::SCollisionForecast temp;

	if (TimeToLine(pos, vel, rad, m_normal, m_distance, temp))
	{
		//intersection might happen to infinite line, but not necessarily to the finite line segment 
		auto contact = glm::dot(temp.contactPoint - m_first, m_ray);
		if (contact >= 0.0f && contact <= m_length) //if contact point is between start and pos;
		{
			if(temp.timeToCollision > 0.0f)
				out = temp;
		}
	}

	if (TimeToPoint(pos, vel, rad, m_first, temp) && temp.timeToCollision < out.timeToCollision)
		out = temp;

	if (TimeToPoint(pos, vel, rad, m_second, temp) && temp.timeToCollision < out.timeToCollision)
		out = temp;

	return out.timeToCollision != INFINITY;
}