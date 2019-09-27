#include <algorithm>

#define SIMULATION_IMPL
#include "CSimulationCuda.cuh"

#include "OdeSolvers.cuh"
#include "CDerivativeSolver.cuh"


using namespace wing2d::simulation;
using namespace wing2d::simulation::cuda;

static float2 TupleToVec(const SimulationState::vec2& v)
{
	return make_float2(std::get<0>(v), std::get<1>(v));
}

std::unique_ptr<ISimulation> wing2d::simulation::cuda::CreateSimulation()
{
	return std::make_unique<CSimulationCuda>();
}

void CSimulationCuda::ResetState(const SimulationState& state)
{
	if (!state.IsValid())
		throw std::runtime_error("state is invalid");
	m_state = state;

	CopyToGPU();

	auto wing = CDerivativeSolver::segments_t();
	auto walls = CDerivativeSolver::segments_t();

	m_odeSolver = std::make_unique<CForwardEulerSolver>(std::make_unique<CDerivativeSolver>(m_state.particles, m_state.particleRad, wing, walls));
}

float CSimulationCuda::Update(float dt)
{
	m_odeSolver->NextState(m_dt, m_curOdeState, m_nextOdeState);
	m_nextOdeState.swap(m_curOdeState);

	return 0.0f;
}

const SimulationState& CSimulationCuda::GetState() const
{
	return m_state;
}

void CSimulationCuda::CopyToGPU()
{
	const size_t& particles = m_state.particles;
		
	OdeStateHost_t posBuf(particles);
	OdeStateHost_t velBuf(particles);

	m_curOdeState.resize(particles * 2);
	m_nextOdeState.resize(particles * 2);

	std::transform(m_state.pos.cbegin(), m_state.pos.cend(), posBuf.begin(), TupleToVec);
	std::transform(m_state.vel.cbegin(), m_state.vel.cend(), velBuf.begin(), TupleToVec);

	thrust::copy_n(posBuf.cbegin(), particles, m_curOdeState.begin());
	thrust::copy_n(velBuf.cbegin(), particles, m_curOdeState.begin() + particles);
}