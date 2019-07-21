#pragma once

#include <vector>
#include <functional>

#include <glm/vec2.hpp>

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class COdeSolver
			{
			public:
				typedef std::vector<glm::vec2> OdeState_t;
				typedef std::function<void(const OdeState_t&, OdeState_t&)> DerivativeSolver;

				void RungeKutta(const OdeState_t& state, DerivativeSolver derivateSolver, float dt, OdeState_t& nextState);
			private:
				OdeState_t m_derivatives[4];
				OdeState_t m_tempState;
			};
		}
	}
}