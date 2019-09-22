#include "pch.hpp"

#include "OdeSolvers.hpp"

using namespace wing2d::simulation;
using namespace wing2d::simulation::cpu;

CForwardEulerSolver::CForwardEulerSolver(std::unique_ptr<IDerivativeSolver>&& derivativeSolver)
{
	if (!derivativeSolver)
		throw std::runtime_error("derivativeSolver is empty");

	m_derivativeSolver = std::move(derivativeSolver);
}

void CForwardEulerSolver::NextState(const OdeState_t& curState, const float dt, OdeState_t& outNextState)
{
	m_derivative.resize(curState.size());
	m_derivativeSolver->Derive(curState, m_derivative);

	std::transform(std::execution::par_unseq, curState.cbegin(), curState.cend(), m_derivative.cbegin(), outNextState.begin(), [&](const auto& s, const auto& v)
		{
			return s + dt * v;
		});
}
