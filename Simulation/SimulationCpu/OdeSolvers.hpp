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
			//class COdeSolver
			//{
			//public:
			//	typedef std::function<void(const OdeState_t&, OdeState_t&)> DerivativeSolver;

			//	void RungeKutta(const OdeState_t& state, DerivativeSolver derivateSolver, float dt, OdeState_t& nextState);
			//	void Euler(const OdeState_t& state, DerivativeSolver derivateSolver, float dt, OdeState_t& nextState);
			//private:
			//	OdeState_t m_derivatives[4];
			//	OdeState_t m_tempState;
			//};

			class CForwardEulerSolver : public IOdeSolver
			{
			public:
				CForwardEulerSolver(std::unique_ptr<IDerivativeSolver>&& derivativeSolver);
				CForwardEulerSolver() = delete;
				CForwardEulerSolver(const CForwardEulerSolver&) = delete;
				CForwardEulerSolver(CForwardEulerSolver&&) = delete;

				virtual void NextState(const OdeState_t& prevState, const OdeState_t& curState, const float dt, OdeState_t& outNextState) override;
			private:
				std::unique_ptr<IDerivativeSolver> m_derivativeSolver = nullptr;
				OdeState_t m_derivative;
			};
		}
	}
} 