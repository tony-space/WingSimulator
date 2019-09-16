#pragma once
#include <glm/glm.hpp>

#include <optional>

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CLineSegment
			{
			public:
				CLineSegment(const glm::vec2& first, const glm::vec2& second);

				CLineSegment() = delete;

				CLineSegment(const CLineSegment&) = default;
				CLineSegment(CLineSegment&&) = default;
				CLineSegment& operator= (const CLineSegment&) = default;

				const glm::vec2& First() const;
				const glm::vec2& Second() const;
				const glm::vec2& Ray() const;
				const glm::vec2& Normal() const;

				float DistanceToLine(const glm::vec2& pos) const;
				glm::vec2 ClosestPoint(const glm::vec2& pos) const;
			private:
				glm::vec2 m_first;
				glm::vec2 m_second;
				glm::vec2 m_ray;
				glm::vec2 m_normal;
				
				//length of the line segment
				float m_length;

				//distance to the center of frame of reference
				float m_distance;
			};
		}
	}
}
