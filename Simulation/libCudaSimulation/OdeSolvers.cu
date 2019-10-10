#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "CudaLaunchHelpers.cuh"
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
	auto elements = curState.size();
	m_derivative.resize(elements);
	m_derivativeSolver->Derive(curState, m_derivative);

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(elements, kBlockSize));

	SaxpyKernel<<<gridDim, blockDim >>> (dt.get(), elements, curState.data().get(), m_derivative.data().get(), outNextState.data().get());
	CudaCheckError();
}