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

	m_center = (a + b + c) / 3.0f;
}

bool CTriangle::IsIntersected(const glm::vec2& pos, const glm::vec2& vel, float rad, glm::vec2& outNormal, float& outDepth) const
{
	auto inBox = false;
	inBox = inBox || m_box.IsInside(glm::vec2(pos.x + rad, pos.y + rad));
	inBox = inBox || m_box.IsInside(glm::vec2(pos.x - rad, pos.y + rad));
	inBox = inBox || m_box.IsInside(glm::vec2(pos.x - rad, pos.y - rad));
	inBox = inBox || m_box.IsInside(glm::vec2(pos.x + rad, pos.y - rad));

	if (!inBox)
		return false;

	constexpr size_t kVertices = 3;

	const CLineSegment* edges[kVertices] = { &m_e1, &m_e2, &m_e3 };
	const glm::vec2* vertices[kVertices] = { &m_e1.origin(), &m_e2.origin(), &m_e3.origin() };
	float absDistances[kVertices] = { FLT_MAX, FLT_MAX, FLT_MAX };
	float sides[kVertices] = { 0.0f, 0.0f, 0.0f };

	for (size_t i = 0; i < kVertices; ++i)
	{
		auto height = edges[i]->DistanceToLine(pos);
		auto distance = height - rad * glm::sign(height);
		absDistances[i] = glm::abs(distance);
		sides[i] = glm::sign(distance);
	}

	if (std::any_of(std::begin(sides), std::end(sides), [](auto side) {return side > 0.0f; }))
		return false;

	auto intersections = std::count_if(std::begin(absDistances), std::end(absDistances), [&](auto absDistance) {return absDistance < rad; });

	switch (intersections)
	{
	case 0:
	case 2:
	case 3:
	{
		auto range = vertices | boost::adaptors::transformed([&](const glm::vec2* p)
		{
			auto delta = *p - pos;
			auto distanceSq = glm::dot(delta, delta);

			return std::make_tuple(distanceSq, *p);
		});

		float distanceSq;
		glm::vec2 vertex;
		std::tie(distanceSq, vertex) = std::reduce(range.begin(), range.end(), *range.begin(), [](const auto& t1, const auto& t2)
		{
			return std::get<0>(t1) < std::get<0>(t2) ? t1 : t2;
		});

		outNormal = vertex - pos;
		outDepth = glm::length(outNormal);
		outNormal /= outDepth;
		break;
	}
	case 1:
	{
		auto minIt = std::min_element(std::begin(absDistances), std::end(absDistances));
		auto index = std::distance(std::begin(absDistances), minIt);

		outNormal = edges[index]->normal();
		outDepth = absDistances[index];

		break;
	}
	}

	return true;
}
