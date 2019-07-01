#include "pch.hpp"

#include "CLineSegment.hpp"

using namespace wing2d::simulation::cpu;

CLineSegment::CLineSegment(const glm::vec2& a, const glm::vec2& b)
{
	auto delta = b - a;
	m_size = glm::length(delta);
	m_origin = a;

	if (m_size > 1e-16f)
		m_ray = delta / m_size;
	else
		m_size = 0.0f;

	m_normal = glm::vec2(m_ray.y, -m_ray.x);
}
