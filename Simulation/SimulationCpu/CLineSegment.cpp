#include "pch.hpp"

#include "CLineSegment.hpp"

using namespace wing2d::simulation::cpu;

CLineSegment::CLineSegment(const glm::vec2& a, const glm::vec2& b)
{
	auto delta = b - a;
	m_length = glm::length(delta);
	m_origin = a;

	if (m_length > 1e-16f)
		m_ray = delta / m_length;
	else
		m_length = 0.0f;

	m_normal = glm::vec2(m_ray.y, -m_ray.x);
	m_distance = glm::dot(a, m_normal);
}

float CLineSegment::DistanceToLine(const glm::vec2& pos) const
{
	auto dirToCenter = pos - m_origin;
	auto centerProj = glm::dot(dirToCenter, m_ray);
	auto isAbove = glm::sign(glm::dot(dirToCenter, m_normal));

	return isAbove * glm::sqrt(glm::dot(dirToCenter, dirToCenter) - centerProj * centerProj);
}