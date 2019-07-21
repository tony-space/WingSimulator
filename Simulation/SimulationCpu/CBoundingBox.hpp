#pragma once

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
				CBoundingBox();
				void AddPoint(const glm::vec2& point);
				bool IsInside(const glm::vec2& point) const;

				template <typename T>
				void AddPoints(T cbegin, T cend)
				{
					for (auto it = cbegin; it != cend; ++it)
						AddPoint(*it);
				}

				const glm::vec2& min()const { return m_min; }
				const glm::vec2& max() const { return m_max; }
			private:
				glm::vec2 m_min;
				glm::vec2 m_max;
			};
		}
	}
}