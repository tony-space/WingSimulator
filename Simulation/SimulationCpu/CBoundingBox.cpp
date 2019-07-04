#include "pch.hpp"

#include "CBoundingBox.hpp"

using namespace wing2d::simulation::cpu;

CBoundingBox::CBoundingBox():
	m_min(FLT_MAX),
	m_max(-FLT_MAX)
{

}

void CBoundingBox::AddPoint(const glm::vec2& point)
{
	m_min = glm::min(m_min, point);
	m_max = glm::max(m_max, point);
}

bool CBoundingBox::IsInside(const glm::vec2& point) const
{
	auto outside = false;

	outside = outside || point.x > m_max.x;
	outside = outside || point.y > m_max.y;
	outside = outside || point.x < m_min.x;
	outside = outside || point.y < m_min.y;

	return !outside;
}