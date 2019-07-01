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
