#pragma once

#include <optional>
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
				void AddBox(const CBoundingBox& other);
				bool IsInside(const glm::vec2& point) const;
				bool Overlaps(const CBoundingBox& other) const;

				template <typename T>
				void AddPoints(T cbegin, T cend)
				{
					for (auto it = cbegin; it != cend; ++it)
						AddPoint(*it);
				}

				const glm::vec2& min()const { return m_min; }
				const glm::vec2& max() const { return m_max; }
				const glm::vec2& center() const;
				const glm::vec2& size() const;
			private:
				std::optional<glm::vec2> m_center;
				std::optional<glm::vec2> m_size;

				glm::vec2 m_min;
				glm::vec2 m_max;
				void UpdateCenter();
				void UpdateSize();
			};
		}
	}
}