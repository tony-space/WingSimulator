#include "pch.hpp"

#include "CTriangle.hpp"

using namespace wing2d::simulation;
using namespace wing2d::simulation::cpu;
using namespace wing2d::simulation::serialization;

CTriangle::CTriangle(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c) :
	m_e1(a, b),
	m_e2(b, c),
	m_e3(c, a)
{
	m_box.AddPoint(a);
	m_box.AddPoint(b);
	m_box.AddPoint(c);
}

bool CTriangle::IsIntersected(const glm::vec2& pos, float rad, glm::vec2& outNormal, float& outDepth) const
{
	auto inBox = false;
	inBox |= m_box.IsInside(glm::vec2(pos.x + rad, pos.y));
	inBox |= m_box.IsInside(glm::vec2(pos.x - rad, pos.y));
	inBox |= m_box.IsInside(glm::vec2(pos.x, pos.y + rad));
	inBox |= m_box.IsInside(glm::vec2(pos.x, pos.y - rad));

	if (!inBox)
		return false;

	const CLineSegment* lines[] = { &m_e1, &m_e2, &m_e3 };

	lines | boost::adaptors::indirected | boost::adaptors::transformed([&](const CLineSegment& line)
	{
		return line.DistanceToLine(pos) - rad;
	});

	return false;
}
