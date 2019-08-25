#include "pch.hpp"

#include "CBoundingBox.hpp"

using namespace wing2d::simulation::cpu;

void CBoundingBox::AddPoint(const glm::vec2& point)
{
	m_min = glm::min(m_min, point);
	m_max = glm::max(m_max, point);
	UpdateCenter();
	UpdateSize();
}

void CBoundingBox::AddPoints(const std::vector<glm::vec2>& points)
{
	auto result = std::transform_reduce(std::execution::par_unseq,
		points.cbegin(), points.cend(), std::make_tuple(INFINITY, INFINITY, -INFINITY, -INFINITY),
		[](const auto& t1, const auto& t2)
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

	UpdateCenter();
	UpdateSize();
}

void CBoundingBox::AddBox(const CBoundingBox& other)
{
	m_min = glm::min(m_min, other.min());
	m_max = glm::max(m_max, other.max());

	UpdateCenter();
	UpdateSize();
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
	const auto& center1 = center();
	const auto& center2 = other.center();

	auto delta = glm::abs(center1 - center2) * 2.0f;
	auto totalSize = size() + other.size();

	return delta.x < totalSize.x && delta.y < totalSize.y;
}

const glm::vec2& CBoundingBox::center() const
{
	return m_center;
}

const glm::vec2 & CBoundingBox::size() const
{
	return m_size;
}

void CBoundingBox::UpdateCenter()
{
	m_center = (m_max + m_min) * 0.5f;
}

void CBoundingBox::UpdateSize()
{
	m_size = m_max - m_min;
}
