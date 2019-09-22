#pragma once

#include <vector>
#include <functional>

#include <glm/vec2.hpp>

#include "CpuInterfaces.h"

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CForwardEulerSolver : public IOdeSolver
			{
			public:
				CForwardEulerSolver(std::unique_ptr<IDerivativeSolver>&& derivativeSolver);
				CForwardEulerSolver() = delete;
				CForwardEulerSolver(const CForwardEulerSolver&) = delete;
				CForwardEulerSolver(CForwardEulerSolver&&) = delete;

				virtual void NextState(const OdeState_t& curState, const float dt, OdeState_t& outNextState) override;
			private:
				std::unique_ptr<IDerivativeSolver> m_derivativeSolver = nullptr;
				OdeState_t m_derivative;
			};
		}
	}
} 