#include "pch.hpp"

#include "COdeSolver.hpp"

using namespace wing2d::simulation::cpu;

void COdeSolver::RungeKutta(const OdeState_t& state, DerivativeSolver derivateSolver, float dt, OdeState_t& nextState)
{
	if (!derivateSolver)
		throw std::runtime_error("derivativeSolver is empty");

	const auto stateLen = state.size();
	for (auto& d : m_derivatives)
		d.resize(stateLen);
	m_tempState.resize(stateLen);
	nextState.resize(stateLen);

	derivateSolver(state, m_derivatives[0]);
	for (auto& v : m_derivatives[0])
		v *= dt;

	for (size_t i = 0; i < stateLen; ++i)
		m_tempState[i] = state[i] + m_derivatives[0][i] * 0.5f;
	derivateSolver(m_tempState, m_derivatives[1]);
	for (auto& v : m_derivatives[1])
		v *= dt;

	for (size_t i = 0; i < stateLen; ++i)
		m_tempState[i] = state[i] + m_derivatives[1][i] * 0.5f;
	derivateSolver(m_tempState, m_derivatives[2]);
	for (auto& v : m_derivatives[2])
		v *= dt;

	for (size_t i = 0; i < stateLen; ++i)
		m_tempState[i] = state[i] + m_derivatives[2][i];
	derivateSolver(m_tempState, m_derivatives[3]);
	for (auto& v : m_derivatives[3])
		v *= dt;

	for (size_t i = 0; i < stateLen; ++i)
		nextState[i] = state[i] + (m_derivatives[0][i] + 2.0f * (m_derivatives[1][i] + m_derivatives[2][i]) + m_derivatives[3][i]) / 6.0f;
}