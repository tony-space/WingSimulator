#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CBoundingBox
			{
			public:
				void AddPoint(const glm::vec2& point);
				void AddPoints(const std::vector<glm::vec2>& points);
				void AddBox(const CBoundingBox& other);

				bool IsInside(const glm::vec2& point) const;
				bool Overlaps(const CBoundingBox& other) const;

				const glm::vec2& min()const { return m_min; }
				const glm::vec2& max() const { return m_max; }
				const glm::vec2& center() const;
				const glm::vec2& size() const;
			private:
				glm::vec2 m_center = glm::vec2(NAN);
				glm::vec2 m_size = glm::vec2(NAN);

				glm::vec2 m_min = glm::vec2(INFINITY);
				glm::vec2 m_max = glm::vec2(-INFINITY);
				void UpdateCenter();
				void UpdateSize();
			};
		}
	}
}