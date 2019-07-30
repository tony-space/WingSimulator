#include "pch.hpp"

#include "CBoundingBox.hpp"

using namespace wing2d::simulation::cpu;

CBoundingBox::CBoundingBox():
	m_min(INFINITY),
	m_max(-INFINITY)
{

}

void CBoundingBox::AddPoint(const glm::vec2& point)
{
	m_min = glm::min(m_min, point);
	m_max = glm::max(m_max, point);
}

void CBoundingBox::AddPoints(const glm::vec2* points, size_t count)
{
	auto horizontal = std::minmax_element(std::execution::par_unseq, points, points + count, [](const auto& p1, const auto& p2)
	{
		return p1.x < p2.x;
	});

	auto vertical = std::minmax_element(std::execution::par_unseq, points, points + count, [](const auto& p1, const auto& p2)
	{
		return p1.y < p2.y;
	});

	AddPoint(glm::vec2(horizontal.first->x, vertical.first->y));
	AddPoint(glm::vec2(horizontal.second->x, vertical.second->y));
}

void CBoundingBox::AddBox(const CBoundingBox& other)
{
	AddPoint(other.min());
	AddPoint(other.max());
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
	if (!m_center.has_value())
		const_cast<CBoundingBox*>(this)->UpdateCenter();
	return m_center.value();
}

const glm::vec2 & CBoundingBox::size() const
{
	if (!m_size.has_value())
		const_cast<CBoundingBox*>(this)->UpdateSize();
	return m_size.value();
}

void CBoundingBox::UpdateCenter()
{
	m_center = (m_max + m_min) * 0.5f;
}

void CBoundingBox::UpdateSize()
{
	m_size = m_max - m_min;
}
