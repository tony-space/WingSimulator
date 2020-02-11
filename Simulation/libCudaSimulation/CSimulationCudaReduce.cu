#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <helper_math.h>

#include <cub/device/device_reduce.cuh>

#include "CudaLaunchHelpers.cuh"
#include "CSimulationCuda.cuh"

using namespace wing2d::simulation::cuda;

static __global__ void ComputeMinTimesKernel(const float2* __restrict__ velocities, size_t particles, float rad, float requestedDt, float* __restrict__ outTimes)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= particles)
		return;

	auto vel = length(velocities[threadId]);
	auto halfRadDt = rad / vel;

	outTimes[threadId] = fminf(halfRadDt, requestedDt);
}

const thrust::device_ptr<float>& CSimulationCuda::ComputeMinDeltaTime(float requestedDt)
{
	const auto particles = m_state.particles;

	m_minDeltaTime.input.resize(particles);

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(particles, kBlockSize));

	ComputeMinTimesKernel<<<gridDim, blockDim>>>(m_curOdeState.data().get() + particles, particles, m_state.particleRad, requestedDt, m_minDeltaTime.input.data().get());

	CudaSafeCall(cudaGetLastError());

	size_t storageBytes = 0;
	CudaSafeCall(cub::DeviceReduce::Min(nullptr, storageBytes, m_minDeltaTime.input.data().get(), m_minDeltaTime.dt.get(), int(particles)));
	
	m_minDeltaTime.cubInternalStorage.resize(storageBytes);

	CudaSafeCall(cub::DeviceReduce::Min(m_minDeltaTime.cubInternalStorage.data().get(), storageBytes, m_minDeltaTime.input.data().get(), m_minDeltaTime.dt.get(), int(particles)));

	return m_minDeltaTime.dt;
}