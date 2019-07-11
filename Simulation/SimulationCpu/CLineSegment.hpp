#pragma once
#include <glm/glm.hpp>

#include "collisions.hpp"

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
				bool PredictCollision(const glm::vec2& pos, const glm::vec2& vel, float rad, collisions::SCollisionForecast& out) const;

				const auto& normal() const { return m_normal; }
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
