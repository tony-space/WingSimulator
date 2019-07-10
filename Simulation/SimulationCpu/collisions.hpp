#pragma once

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			namespace collisions
			{
				struct SCollisionForecast
				{
					glm::vec2 contactPoint;
					glm::vec2 normal;
					float timeToCollision;
				};

				bool TimeToPoint(const glm::vec2& pos, const glm::vec2& vel, float rad, const glm::vec2& point, SCollisionForecast& out);
				bool TimeToPlane(const glm::vec2& pos, const glm::vec2& vel, float rad, const glm::vec2& planeNormal, float planeDistance, SCollisionForecast& out);
			}
		}
	}
}
