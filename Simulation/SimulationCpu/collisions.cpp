#include "pch.hpp"

#include "collisions.hpp"

bool wing2d::simulation::cpu::collisions::TimeToPoint(const glm::vec2& pos, const glm::vec2& vel, float rad, const glm::vec2& point, SCollisionForecast& out)
{
	//Time prediction based on the following pair of vector equations
	//C0 + Vt = C1
	//(P - C1) dot (P - C1) = r^2

	auto deltaPos = pos - point;
	auto a = glm::dot(vel, vel);
	auto b = 2.0f * glm::dot(vel, deltaPos);
	auto c = glm::dot(deltaPos, deltaPos) - rad * rad;

	//if particle is almost static
	if (glm::abs(a) < 1e-16)
		return false;

	//if particle is flying away from the point
	if (b > 0.0f)
		return false;

	auto D = b * b - 4 * a * c;
	if (D < 0)
		return false;
	auto sqrtD = glm::sqrt(D);
	auto t = (-b - sqrtD) / (2.0f * a);

	assert(t >= 0.0f);

	out.timeToCollision = t;
	out.contactPoint = point;
	out.normal = glm::normalize(deltaPos);

	return true;
}

bool wing2d::simulation::cpu::collisions::TimeToLine(const glm::vec2& pos, const glm::vec2& vel, float rad, glm::vec2 lineNormal, float lineDistance, SCollisionForecast& out)
{
	auto distanceToLine = glm::dot(pos, lineNormal) - lineDistance;
	if (distanceToLine < 0.0f)
	{
		distanceToLine *= -1.0f;
		lineNormal *= -1.0f;
	}

	auto approachVel = -glm::dot(vel, lineNormal);
	
	//if almost zero or negative
	if (approachVel < 1e-16f)
		return false;

	//The distance to the line may become negative.
	//That means, the particle intersects infinite line,
	//but that doesn't necessarily mean that the particle intersects the line segment
	distanceToLine -= rad;

	out.timeToCollision = distanceToLine / approachVel;
	out.normal = lineNormal;
	out.contactPoint = pos + vel * out.timeToCollision - lineNormal * rad;

	return true;
}