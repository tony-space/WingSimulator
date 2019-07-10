#pragma once
#include <glm/glm.hpp>

#include "CBoundingBox.hpp"

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

				float DistanceToLine(const glm::vec2& pos) const;

				const auto& normal() const { return m_normal; }
				const auto& origin() const { return m_origin;  }
			private:
				glm::vec2 m_origin;
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
