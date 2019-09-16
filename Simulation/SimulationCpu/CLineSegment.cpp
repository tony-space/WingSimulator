#include "pch.hpp"
#include "CLineSegment.hpp"

using namespace wing2d::simulation::cpu;

CLineSegment::CLineSegment(const glm::vec2& a, const glm::vec2& b) : m_ray()
{
	auto delta = b - a;
	m_length = glm::length(delta);
	m_first = a;
	m_second = b;

	if (m_length > 1e-16f)
		m_ray = delta / m_length;
	else
		m_length = 0.0f;

	m_normal = glm::vec2(-m_ray.y, m_ray.x);
	m_distance = glm::dot(a, m_normal);
}

const glm::vec2& wing2d::simulation::cpu::CLineSegment::First() const
{
	return m_first;
}

const glm::vec2& wing2d::simulation::cpu::CLineSegment::Second() const
{
	return m_second;
}

const glm::vec2& wing2d::simulation::cpu::CLineSegment::Ray() const
{
	return m_ray;
}

const glm::vec2& CLineSegment::Normal() const
{
	return m_normal; 
}

float CLineSegment::DistanceToLine(const glm::vec2& pos) const
{
	auto dirToCenter = pos - m_first;
	auto centerProj = glm::dot(dirToCenter, m_ray);
	auto isAbove = glm::sign(glm::dot(dirToCenter, m_normal));

	return isAbove * glm::sqrt(glm::dot(dirToCenter, dirToCenter) - centerProj * centerProj);
}

glm::vec2 CLineSegment::ClosestPoint(const glm::vec2& pos) const
{
	const auto toPos = pos - m_first;
	const auto projection = glm::dot(m_ray, toPos);

	if (projection < 0.0f)
	{
		return m_first;
	}
	else if (projection >= 0.0f && projection <= m_length)
	{
		return m_first + m_ray * projection;
	}
	else
	{
		return m_second;
	}
}
