#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "../Simulation.hpp"

#include "CBoundingBox.hpp"
#include "CLineSegment.hpp"

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CTriangle
			{
			public:
				CTriangle(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c);
			private:
				CBoundingBox m_box;

				CLineSegment m_e1;
				CLineSegment m_e2;
				CLineSegment m_e3;
			};
		}
	}
}