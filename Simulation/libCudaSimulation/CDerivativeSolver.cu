#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "CDerivativeSolver.cuh"

static __global__ void AddGravityKernel(float2* forces, unsigned n)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= n)
		return;

	forces[threadId].x += 0.5f;
}

using namespace wing2d::simulation::cuda;

CDerivativeSolver::CDerivativeSolver(size_t particles, float radius, const Segments_t& airfoil, const Segments_t& walls) :
	m_airfoilStorage(airfoil),
	m_wallsStorage(walls),
	m_forces(particles),
	m_particles(particles),
	m_particleRad(radius)
{

}

void CDerivativeSolver::Derive(const OdeState_t& curState, OdeState_t& outDerivative)
{
	ResetForces();
	BuildParticlesTree(curState);
	ResolveParticleParticleCollisions(curState);
	ResolveParticleWingCollisions(curState);
	ParticleToWall(curState);
	ApplyGravity();
	BuildDerivative(curState, outDerivative);
}

void CDerivativeSolver::ResetForces()
{
	auto devPtr = m_forces.data().get();
	auto bytesSize = m_forces.size() * sizeof(decltype(m_forces)::value_type);
	cudaMemsetAsync(devPtr, 0, bytesSize);
}

void CDerivativeSolver::BuildParticlesTree(const OdeState_t& curState)
{

}

void CDerivativeSolver::ResolveParticleParticleCollisions(const OdeState_t& curState)
{

}

void CDerivativeSolver::ResolveParticleWingCollisions(const OdeState_t& curState)
{

}

void CDerivativeSolver::ParticleToWall(const OdeState_t& curState)
{

}

void CDerivativeSolver::ApplyGravity()
{
	auto elements = unsigned(m_forces.size());

	dim3 blockDim(64);
	dim3 gridDim((elements - 1) / blockDim.x + 1);

	AddGravityKernel <<<gridDim, blockDim >>> (m_forces.data().get(), elements);
}

void CDerivativeSolver::BuildDerivative(const OdeState_t& curState, OdeState_t& outDerivative) const
{
	const float2* d_velocities = curState.data().get() + m_particles;
	const float2* d_forces = m_forces.data().get();
	float2* d_derivative = outDerivative.data().get();

	const size_t dataBlockSize = m_particles * sizeof(float2);

	cudaMemcpyAsync(d_derivative, d_velocities, dataBlockSize, cudaMemcpyKind::cudaMemcpyDeviceToDevice);
	cudaMemcpyAsync(d_derivative + m_particles, d_forces, dataBlockSize, cudaMemcpyKind::cudaMemcpyDeviceToDevice);
}