#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <algorithm>

#define SIMULATION_IMPL
#include "CSimulationCuda.cuh"

#include "OdeSolvers.cuh"
#include "CudaLaunchHelpers.cuh"


using namespace wing2d::simulation;
using namespace wing2d::simulation::cuda;

static __device__ float4 GetHeatMapColor(float value)
{
	value = fminf(fmaxf(value, 0.0f), 1.0f);

	static const size_t stages = 7;
	static const float3 heatMap[stages] = { {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f} };
	value *= stages - 1;
	int idx1 = int(value);
	int idx2 = idx1 + 1;
	float fract1 = value - float(idx1);
	auto result = heatMap[idx1] + fract1 * (heatMap[idx2] - heatMap[idx1]);

	return make_float4(result, 1.0f);
}

static __global__ void ColorParticlesKernel(const float* __restrict__ pDt, const size_t nParticles, const float2* __restrict__ lastVel, const float2* __restrict__ nextVel, float4* __restrict__ color)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= nParticles)
		return;

	auto force = (nextVel[threadId] - lastVel[threadId]) / *pDt;
	color[threadId] = GetHeatMapColor(logf(length(force) + 1.0f) / 10.0f + 0.15f);
}

static __global__ void ColorParticlesKernel2(const size_t nParticles, const float* __restrict__ pressures, const TIndex* __restrict__ oldIndices, float4* __restrict__ color)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= nParticles)
		return;

	auto oldIdx = oldIndices[threadId];
	auto pressure = pressures[threadId];

	color[oldIdx] = GetHeatMapColor(logf(pressure + 1.0f) / 10.0f + 0.15f);
}


static float2 TupleToVec(const SimulationState::vec2& v)
{
	return make_float2(std::get<0>(v), std::get<1>(v));
}


static SimulationState::vec2 VecToTuple2D(const float2& v)
{
	return std::make_tuple(v.x, v.y);
};

static SimulationState::vec4 VecToTuple4D(const float4& v)
{
	return std::make_tuple(v.x, v.y, v.z, v.w);
};

static Segments_t BuildWalls(const SimulationState& state)
{
	auto corner = make_float2(state.worldSize.width / 2.0f, state.worldSize.height / 2.0f);
	auto topLeft = make_float2(-corner.x, corner.y);
	auto topRight = make_float2(corner.x, corner.y);
	auto bottomRight = make_float2(corner.x, -corner.y);
	auto bottomLeft = make_float2(-corner.x, -corner.y);

	Segments_t result;

	result.emplace_back(std::make_tuple(topRight, topLeft));
	result.emplace_back(std::make_tuple(bottomRight, topRight));
	result.emplace_back(std::make_tuple(bottomLeft, bottomRight));
	result.emplace_back(std::make_tuple(topLeft, bottomLeft));

	return result;
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

	m_derivativeSolver = std::make_unique<CDerivativeSolver>(m_state.particles, m_state.particleRad, Segments_t(), BuildWalls(m_state));
	m_odeSolver = std::make_unique<CForwardEulerSolver>(m_derivativeSolver.get());
}

float CSimulationCuda::Update(float dt)
{
	for (int i = 0; i < 16; ++i)
	{
		m_odeSolver->NextState(ComputeMinDeltaTime(dt), m_curOdeState, m_nextOdeState);
		m_nextOdeState.swap(m_curOdeState);
	}
	
	//ColorParticles2();
	return dt;
}

void CSimulationCuda::ColorParticles()
{
	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(m_state.particles, kBlockSize));

	const float2* lastVel = m_curOdeState.data().get() + m_state.particles;
	const float2* nextVel = m_nextOdeState.data().get() + m_state.particles;
	float4* colors = m_deviceColors.data().get();

	ColorParticlesKernel <<<gridDim, blockDim >>> (m_minDeltaTime.dt.get(), m_state.particles, lastVel, nextVel, colors);
	CudaCheckError();

	m_hostColors = m_deviceColors;
	std::transform(m_hostColors.cbegin(), m_hostColors.cend(), m_state.color.begin(), VecToTuple4D);
}

void CSimulationCuda::ColorParticles2()
{
	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(m_state.particles, kBlockSize));

	auto pressures = m_derivativeSolver->GetPressures().data().get();
	auto oldIndices = m_derivativeSolver->GetParticlesIndices().data().get();
	auto resultColors = m_deviceColors.data().get();

	ColorParticlesKernel2 <<<gridDim, blockDim >>> (m_state.particles, pressures, oldIndices, resultColors);
	CudaCheckError();

	m_hostColors = m_deviceColors;
	std::transform(m_hostColors.cbegin(), m_hostColors.cend(), m_state.color.begin(), VecToTuple4D);
}


const SimulationState& CSimulationCuda::GetState()
{
	m_hostOdeState = m_curOdeState;
	std::transform(m_hostOdeState.cbegin(), m_hostOdeState.cbegin() + m_state.particles, m_state.pos.begin(), VecToTuple2D);
	std::transform(m_hostOdeState.cbegin() + m_state.particles, m_hostOdeState.cend(), m_state.vel.begin(), VecToTuple2D);

	return m_state;
}

void CSimulationCuda::CopyToGPU()
{
	const size_t& particles = m_state.particles;

	PinnedHostVector2D_t posBuf(particles);
	PinnedHostVector2D_t velBuf(particles);

	m_curOdeState.resize(particles * 2);
	m_nextOdeState.resize(particles * 2);
	m_hostOdeState.resize(particles * 2);
	
	m_deviceColors.resize(particles);
	m_hostColors.resize(particles);

	std::transform(m_state.pos.cbegin(), m_state.pos.cend(), posBuf.begin(), TupleToVec);
	std::transform(m_state.vel.cbegin(), m_state.vel.cend(), velBuf.begin(), TupleToVec);

	thrust::copy_n(posBuf.cbegin(), particles, m_curOdeState.begin());
	thrust::copy_n(velBuf.cbegin(), particles, m_curOdeState.begin() + particles);
}