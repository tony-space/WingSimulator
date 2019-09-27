#pragma once

#include <vector>
#include <glm/vec2.hpp>

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			typedef std::vector<glm::vec2> OdeState_t;

			struct IDerivativeSolver
			{
				virtual void Derive(const OdeState_t& curState, OdeState_t& outDerivative) = 0;
				virtual ~IDerivativeSolver() = default;
			};

			struct IOdeSolver
			{
				virtual void NextState(const OdeState_t& curState, const float dt, OdeState_t& outNextState) = 0;
				virtual ~IOdeSolver() = default;
			};
		}
	}
}