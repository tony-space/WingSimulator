#include "pch.hpp"

#include "OdeSolvers.hpp"

using namespace wing2d::simulation;
using namespace wing2d::simulation::cpu;

//void COdeSolver::RungeKutta(const OdeState_t& state, DerivativeSolver derivateSolver, float dt, OdeState_t& nextState)
//{
//	if (!derivateSolver)
//		throw std::runtime_error("derivativeSolver is empty");
//
//	const auto stateLen = state.size();
//	for (auto& d : m_derivatives)
//		d.resize(stateLen);
//	m_tempState.resize(stateLen);
//	nextState.resize(stateLen);
//
//	derivateSolver(state, m_derivatives[0]);
//	std::transform(std::execution::par_unseq, m_derivatives[0].cbegin(), m_derivatives[0].cend(), m_derivatives[0].begin(), [&](const auto& v)
//	{
//		return v * dt;
//	});
//	std::transform(std::execution::par_unseq, m_derivatives[0].cbegin(), m_derivatives[0].cend(), state.cbegin(), m_tempState.begin(), [&](const auto& a, const auto& b)
//	{
//		return a * 0.5f + b;
//	});
//
//	derivateSolver(m_tempState, m_derivatives[1]);
//	std::transform(std::execution::par_unseq, m_derivatives[1].cbegin(), m_derivatives[1].cend(), m_derivatives[1].begin(), [&](const auto& v)
//	{
//		return v * dt;
//	});
//	std::transform(std::execution::par_unseq, m_derivatives[1].cbegin(), m_derivatives[1].cend(), state.cbegin(), m_tempState.begin(), [](const auto& a, const auto& b)
//	{
//		return a * 0.5f + b;
//	});
//
//	derivateSolver(m_tempState, m_derivatives[2]);
//
//	std::transform(std::execution::par_unseq, m_derivatives[2].cbegin(), m_derivatives[2].cend(), m_derivatives[2].begin(), [&](const auto& v)
//	{
//		return v * dt;
//	});
//	std::transform(std::execution::par_unseq, m_derivatives[2].cbegin(), m_derivatives[2].cend(), state.cbegin(), m_tempState.begin(), [](const auto& a, const auto& b)
//	{
//		return a + b;
//	});
//
//	derivateSolver(m_tempState, m_derivatives[3]);
//
//	std::transform(std::execution::par_unseq, m_derivatives[3].cbegin(), m_derivatives[3].cend(), m_derivatives[3].begin(), [&](const auto& v)
//	{
//		return v * dt;
//	});
//
//	std::transform(std::execution::par_unseq, nextState.cbegin(), nextState.cend(), nextState.begin(), [&](const auto& next)
//	{
//		auto i = &next - nextState.data();
//		return state[i] + (m_derivatives[0][i] + 2.0f * (m_derivatives[1][i] + m_derivatives[2][i]) + m_derivatives[3][i]) / 6.0f;
//	});
//}

CForwardEulerSolver::CForwardEulerSolver(std::unique_ptr<IDerivativeSolver>&& derivativeSolver)
{
	if (!derivativeSolver)
		throw std::runtime_error("derivativeSolver is empty");

	m_derivativeSolver = std::move(derivativeSolver);
}

void CForwardEulerSolver::NextState(const OdeState_t& prevState, const OdeState_t& curState, const float dt, OdeState_t& outNextState)
{
	m_derivative.resize(curState.size());
	m_derivativeSolver->Derive(prevState, curState, m_derivative);

	std::transform(std::execution::par_unseq, curState.cbegin(), curState.cend(), m_derivative.cbegin(), outNextState.begin(), [&](const auto& s, const auto& v)
		{
			return s + dt * v;
		});
}
