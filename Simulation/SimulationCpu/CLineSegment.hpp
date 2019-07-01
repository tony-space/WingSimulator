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
			private:
				glm::vec2 m_origin;
				glm::vec2 m_ray;
				glm::vec2 m_normal;
				float m_size;
				float __padding;
			};
		}
	}
}
