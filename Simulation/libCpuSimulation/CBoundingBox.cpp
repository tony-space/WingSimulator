#include "pch.hpp"

#include "CBoundingBox.hpp"

using namespace wing2d::simulation::cpu;

CBoundingBox::CBoundingBox(const glm::vec2& p1, const glm::vec2& p2)
{
	m_min = glm::min(p1, p2);
	m_max = glm::max(p1, p2);
}

void CBoundingBox::AddPoint(const glm::vec2& point)
{
	m_min = glm::min(m_min, point);
	m_max = glm::max(m_max, point);
}

void CBoundingBox::AddPoints(const std::vector<glm::vec2>& points)
{
	auto result = std::transform_reduce(std::execution::par_unseq, points.cbegin(), points.cend(), std::make_tuple(INFINITY, INFINITY, -INFINITY, -INFINITY), [](const auto& t1, const auto& t2)
	{
		return std::make_tuple(
			std::min(std::get<0>(t1), std::get<0>(t2)),
			std::min(std::get<1>(t1), std::get<1>(t2)),
			std::max(std::get<2>(t1), std::get<2>(t2)),
			std::max(std::get<3>(t1), std::get<3>(t2))
		);
	}, [](const auto& p)
	{
		return std::make_tuple(p.x, p.y, p.x, p.y);
	});

	glm::vec2 minPoint(std::get<0>(result), std::get<1>(result));
	glm::vec2 maxPoint(std::get<2>(result), std::get<3>(result));

	m_min = glm::min(m_min, minPoint);
	m_max = glm::max(m_max, maxPoint);
}

void CBoundingBox::AddBox(const CBoundingBox& box)
{
	m_min = glm::min(m_min, box.Min());
	m_max = glm::max(m_max, box.Max());
}

void CBoundingBox::AddBoxes(const std::vector<CBoundingBox>& boxes)
{
	auto result = std::transform_reduce(std::execution::par_unseq, boxes.cbegin(), boxes.cend(), std::make_tuple(INFINITY, INFINITY, -INFINITY, -INFINITY), [](const auto& t1, const auto& t2)
	{
		return std::make_tuple(
			std::min(std::get<0>(t1), std::get<0>(t2)),
			std::min(std::get<1>(t1), std::get<1>(t2)),
			std::max(std::get<2>(t1), std::get<2>(t2)),
			std::max(std::get<3>(t1), std::get<3>(t2))
		);
	}, [](const CBoundingBox& box)
	{
		const auto& min = box.Min();
		const auto& max = box.Max();
		return std::make_tuple(min.x, min.y, max.x, max.y);
	});

	glm::vec2 minPoint(std::get<0>(result), std::get<1>(result));
	glm::vec2 maxPoint(std::get<2>(result), std::get<3>(result));

	m_min = glm::min(m_min, minPoint);
	m_max = glm::max(m_max, maxPoint);
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

bool CBoundingBox::Overlaps(const CBoundingBox& other) const
{
	const auto& center1 = Center();
	const auto& center2 = other.Center();

	auto delta = glm::abs(center1 - center2) * 2.0f;
	auto totalSize = Size() + other.Size();

	return delta.x < totalSize.x && delta.y < totalSize.y;
}

void CBoundingBox::SetMin(const glm::vec2& min)
{
	m_min = min;
}

void CBoundingBox::SetMax(const glm::vec2& max)
{
	m_max = max;
}

const glm::vec2& CBoundingBox::Min() const
{
	return m_min;
}

const glm::vec2& CBoundingBox::Max() const
{
	return m_max;
}

const glm::vec2 CBoundingBox::Center() const
{
	return (m_max + m_min) * 0.5f;
}

const glm::vec2 CBoundingBox::Size() const
{
	return m_max - m_min;
}