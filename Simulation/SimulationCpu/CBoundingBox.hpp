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

				template <typename T>
				void AddPoints(T cbegin, T cend)
				{
					for (auto it = cbegin; it != cend; ++it)
						AddPoint(*it);
				}

				template <typename T>
				void AddPoints(const T& collection)
				{
					auto cbegin = collection.cbegin();
					auto cend = collection.cend();
					AddPoints(cbegin, cend);
				}
			private:
				glm::vec2 m_min;
				glm::vec2 m_max;
			};
		}
	}
}