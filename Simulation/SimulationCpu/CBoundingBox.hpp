#pragma once

#include <vector>
#include <glm/glm.hpp>

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CBoundingBox
			{
			public:
				CBoundingBox() = default;
				CBoundingBox(const glm::vec2& p1, const glm::vec2& p2);
				CBoundingBox(const CBoundingBox&) = default;
				CBoundingBox(CBoundingBox&&) = default;


				void AddPoint(const glm::vec2& point);
				void AddPoints(const std::vector<glm::vec2>& points);
				
				void AddBox(const CBoundingBox& box);
				void AddBoxes(const std::vector<CBoundingBox>& boxes);

				bool IsInside(const glm::vec2& point) const;
				bool Overlaps(const CBoundingBox& other) const;

				void SetMin(const glm::vec2& min);
				void SetMax(const glm::vec2& max);

				const glm::vec2& Min()const;
				const glm::vec2& Max() const;

				const glm::vec2 Center() const;
				const glm::vec2 Size() const;
			private:
				glm::vec2 m_min = glm::vec2(INFINITY);
				glm::vec2 m_max = glm::vec2(-INFINITY);
			};
		}
	}
}