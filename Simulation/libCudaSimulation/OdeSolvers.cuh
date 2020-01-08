#pragma once

#include <memory>

#include "CudaInterfaces.cuh"

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			class CForwardEulerSolver : public IOdeSolver
			{
			public:
				CForwardEulerSolver(IDerivativeSolver* derivativeSolver);
				CForwardEulerSolver() = delete;
				CForwardEulerSolver(const CForwardEulerSolver&) = delete;
				CForwardEulerSolver(CForwardEulerSolver&&) = delete;

				virtual void NextState(const thrust::device_ptr<float>& dt, const OdeState_t& curState, OdeState_t& outNextState) override;
			private:
				IDerivativeSolver* m_derivativeSolver = nullptr;
				OdeState_t m_derivative;
			};
		}
	}
}