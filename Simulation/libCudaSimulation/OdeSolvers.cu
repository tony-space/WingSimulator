#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "OdeSolvers.cuh"

using namespace wing2d::simulation;
using namespace wing2d::simulation::cuda;

static __global__ void SaxpyKernel(const float* __restrict__ pDt, const size_t nElements, const float2* __restrict__ state, const float2* __restrict__ derivative, float2* __restrict__ nextState)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	const auto dt = *pDt;

	if (threadId >= nElements)
		return;

	nextState[threadId] = state[threadId] + derivative[threadId] * dt;
}

CForwardEulerSolver::CForwardEulerSolver(std::unique_ptr<IDerivativeSolver>&& derivativeSolver)
{
	if (!derivativeSolver)
		throw std::runtime_error("derivativeSolver is empty");

	m_derivativeSolver = std::move(derivativeSolver);
}

void CForwardEulerSolver::NextState(const thrust::device_ptr<float>& dt, const OdeState_t& curState, OdeState_t& outNextState)
{
	m_derivative.resize(curState.size());
	m_derivativeSolver->Derive(curState, m_derivative);

	auto elements = unsigned(curState.size());

	dim3 blockDim(64);
	dim3 gridDim((elements - 1) / blockDim.x + 1);

	SaxpyKernel<<<gridDim, blockDim >>> (dt.get(), elements, curState.data().get(), m_derivative.data().get(), outNextState.data().get());
}