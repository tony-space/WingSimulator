#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "CudaLaunchHelpers.cuh"
#include "CDerivativeSolver.cuh"

using namespace wing2d::simulation::cuda;

static __device__ float2 SpringDamper(const float2& normal, const float2& vel1, const float2& vel2, float springLen)
{
	constexpr float stiffness = 10000.0f;
	constexpr float damp = 50.0f;
	auto v = dot(vel1 - vel2, normal);
	return normal * (springLen * stiffness + v * damp) * -1.0f;
}

static __device__ float2 SpringDamper2(const float2& normal, const float2& vel1, const float2& vel2, float springLen)
{
	constexpr float stiffness = 50000.0f;
	constexpr float damp = 50.0f;
	auto v = dot(vel1 - vel2, normal);
	return normal * (springLen * stiffness + v * damp) * -1.0f;
}

static __global__ void ParticleToWallKernel(const size_t particles, const float radius, const float2* __restrict__ pOdeState, SLineSegmentsSOA walls, float2* __restrict__ outForces)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= particles)
		return;

	const auto pos = pOdeState[threadId];
	const auto vel = pOdeState[threadId + particles];
	
	auto force = outForces[threadId];

	for (size_t i = 0; i < walls.lineSegments; ++i)
	{
		const auto d = walls.DistanceToLine(i, pos) - radius;
		if (d < 0.0f)
		{
			force += SpringDamper(walls.normal[i], vel, make_float2(0.0f), d);
		}
	}

	outForces[threadId] = force;
}

static __global__ void AddGravityKernel(float2* forces, unsigned n)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= n)
		return;

	forces[threadId].x += 0.5f;
}

static __global__ void BuildAirfoilBoxesKernel(SLineSegmentsSOA airfoil, SBoundingBoxesSOA boxes)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= airfoil.lineSegments)
		return;

	const auto f = airfoil.first[threadId];
	const auto s = airfoil.second[threadId];

	const auto minCorner = fminf(f, s);
	const auto maxCorner = fmaxf(f, s);

	boxes.min[threadId] = minCorner;
	boxes.max[threadId] = maxCorner;
}

static __global__ void BuildParticlesBoundingBoxesKernel(SBoundingBoxesSOA boundingBoxes, const float particleRad, const float2* __restrict__ particlePos)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= boundingBoxes.boundingBoxes)
		return;

	auto pos = particlePos[threadId];
	auto minCorner = make_float2(pos.x - particleRad, pos.y - particleRad);
	auto maxCorner = make_float2(pos.x + particleRad, pos.y + particleRad);

	boundingBoxes.min[threadId] = minCorner;
	boundingBoxes.max[threadId] = maxCorner;
}

static __global__ void ResolveParticleParticleCollisionsKernel(const CMortonTree::SDeviceCollisions potentialCollisions, CDerivativeSolver::SIntermediateSimState simState)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= simState.particles)
		return;

	const auto pos1 = simState.pos[threadId];
	const auto vel1 = simState.vel[threadId];
	const auto diameter = simState.particleRad * 2.0f;
	const auto diameterSq = diameter * diameter;
	auto totalForce = make_float2(0.0f);
	auto totalPressure = 0.0f;

	for (size_t collisionIdx = 0; collisionIdx < potentialCollisions.maxCollisionsPerElement; ++collisionIdx)
	{
		const auto otherParticleIdx = potentialCollisions.internalIndices[collisionIdx * potentialCollisions.elements + threadId];
		if (otherParticleIdx == threadId)
			continue;
		if (otherParticleIdx == size_t(-1))
			break;

		const auto pos2 = simState.pos[otherParticleIdx];
		const auto deltaPos = pos2 - pos1;
		const auto distanceSq = dot(deltaPos, deltaPos);
		if (distanceSq > diameterSq || distanceSq < 1e-8f)
			continue;

		const auto vel2 = simState.vel[otherParticleIdx];

		auto dist = sqrtf(distanceSq);
		auto dir = deltaPos / dist;
		auto springLen = diameter - dist;

		auto force = SpringDamper(dir, vel1, vel2, springLen);
		totalForce += force;
		totalPressure += length(force);
	}

	simState.force[threadId] += totalForce;
	simState.pressure[threadId] += totalPressure;
}

static __global__ void ResolveParticleWingCollisionsKernel(const CMortonTree::SDeviceCollisions potentialCollisions, CDerivativeSolver::SIntermediateSimState simState, SLineSegmentsSOA airfoil)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= simState.particles)
		return;

	const auto pos = simState.pos[threadId];
	struct SSegment
	{
		float2 first;
		float2 ray;
		float2 normal;

		float length = 0.0f;
		float distanceTo = INFINITY;
	} closest;

	for (size_t collisionIdx = 0; collisionIdx < potentialCollisions.maxCollisionsPerElement; ++collisionIdx)
	{
		const auto wingSegmentIdx = potentialCollisions.internalIndices[collisionIdx * potentialCollisions.elements + threadId];
		if (wingSegmentIdx == size_t(-1))
			break;

		SSegment cur =
		{
			airfoil.first[wingSegmentIdx],
			airfoil.ray[wingSegmentIdx],
			airfoil.normal[wingSegmentIdx],
			airfoil.length[wingSegmentIdx],
			INFINITY
		};

		const auto projection = dot(cur.ray, pos - cur.first);

		float2 closestPoint;
		if (projection < 0.0f)
		{
			closestPoint = cur.first;
		}
		else if (projection >= 0.0f && projection <= cur.length)
		{
			closestPoint = cur.first + cur.ray * projection;
		}
		else
		{
			closestPoint = cur.first + cur.ray * cur.length;
		}

		cur.distanceTo = length(pos - closestPoint);
		
		if (cur.distanceTo < closest.distanceTo)
		{
			closest = cur;
		}
		else if (cur.distanceTo == closest.distanceTo)
		{
			auto delta1 = pos - closest.first;
			auto delta2 = pos - cur.first;

			auto dot1 = dot(delta1, closest.normal);
			auto dot2 = dot(delta2, cur.normal);

			if (dot2 > dot1)
				closest = cur;
		}
	}

	if (closest.distanceTo == INFINITY)
		return;

	const auto height = dot(closest.normal, pos - closest.first);

	if (height <= simState.particleRad)
	{
		const auto penetration = height - simState.particleRad;
		auto force = SpringDamper2(closest.normal, simState.vel[threadId], make_float2(0.0f), penetration);
		auto pressure = length(force);
		simState.force[threadId] += force;
		simState.pressure[threadId] += pressure;
	}
}

//
//
//
CDerivativeSolver::CDerivativeSolver(size_t particles, float radius, const Segments_t& airfoil, const Segments_t& walls) :
	m_airfoilStorage(airfoil),
	m_wallsStorage(walls),
	m_airfoilsBoxesStorage(airfoil.size()),
	m_particlesBoxesStorage(particles),
	m_particlesExtendedBoxesStorage(particles),
	m_forces(particles),
	m_pressures(particles),
	m_particles(particles),
	m_particleRad(radius)
{
	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(airfoil.size(), kBlockSize));
	auto boxesStorage = m_airfoilsBoxesStorage.get();
	BuildAirfoilBoxesKernel <<<gridDim, blockDim >>> (m_airfoilStorage.get(), boxesStorage);
	CudaCheckError();
	m_airfoilTree.Build(boxesStorage);
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

CDerivativeSolver::SIntermediateSimState CDerivativeSolver::GetSimState(const OdeState_t& curState)
{
	return
	{
		m_particles,
		m_particleRad,
		curState.data().get(),
		curState.data().get() + m_particles,
		m_forces.data().get(),
		m_pressures.data().get()
	};
}

void CDerivativeSolver::ResetForces()
{
	CudaSafeCall(cudaMemsetAsync(m_forces.data().get(), 0, m_particles * sizeof(float2)));
	CudaSafeCall(cudaMemsetAsync(m_pressures.data().get(), 0, m_particles * sizeof(float)));
}

void CDerivativeSolver::BuildParticlesTree(const OdeState_t& curState)
{
	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(m_particles, kBlockSize));


	auto boxesStorage = m_particlesBoxesStorage.get();

	BuildParticlesBoundingBoxesKernel <<<gridDim, blockDim >>>(boxesStorage, m_particleRad, curState.data().get());
	BuildParticlesBoundingBoxesKernel <<<gridDim, blockDim >>>(m_particlesExtendedBoxesStorage.get(), m_particleRad * 4.0f, curState.data().get());
	CudaCheckError();

	m_particlesTree.Build(boxesStorage);
}

void CDerivativeSolver::ResolveParticleParticleCollisions(const OdeState_t& curState)
{
	const auto collisionsResult = m_particlesTree.TraverseReflexive(128);

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(m_particles, kBlockSize));

	ResolveParticleParticleCollisionsKernel <<<gridDim, blockDim >>> (collisionsResult, GetSimState(curState));

	CudaCheckError();

}

void CDerivativeSolver::ResolveParticleWingCollisions(const OdeState_t& curState)
{
	const auto collisionsResult = m_airfoilTree.Traverse(m_particlesExtendedBoxesStorage.get());

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(m_particles, kBlockSize));

	auto simState = GetSimState(curState);
	ResolveParticleWingCollisionsKernel <<<gridDim, blockDim >>> (collisionsResult, simState, m_airfoilStorage.get());

	CudaCheckError();
}

void CDerivativeSolver::ParticleToWall(const OdeState_t& curState)
{
	auto elements = unsigned(m_particles);
	dim3 blockDim(kBlockSize);
	dim3 gridDim((elements - 1) / blockDim.x + 1);

	ParticleToWallKernel <<<gridDim, blockDim >>> (m_particles, m_particleRad, curState.data().get(), m_wallsStorage.get(), m_forces.data().get());
	CudaCheckError();
}

void CDerivativeSolver::ApplyGravity()
{
	auto elements = unsigned(m_particles);
	dim3 blockDim(kBlockSize);
	dim3 gridDim((elements - 1) / blockDim.x + 1);

	AddGravityKernel <<<gridDim, blockDim >>> (m_forces.data().get(), elements);
	CudaCheckError();
}

void CDerivativeSolver::BuildDerivative(const OdeState_t& curState, OdeState_t& outDerivative) const
{
	const float2* d_velocities = curState.data().get() + m_particles;
	const float2* d_forces = m_forces.data().get();
	float2* d_derivative = outDerivative.data().get();

	const size_t dataBlockSize = m_particles * sizeof(float2);

	CudaSafeCall(cudaMemcpyAsync(d_derivative, d_velocities, dataBlockSize, cudaMemcpyKind::cudaMemcpyDeviceToDevice));
	CudaSafeCall(cudaMemcpyAsync(d_derivative + m_particles, d_forces, dataBlockSize, cudaMemcpyKind::cudaMemcpyDeviceToDevice));
}

const thrust::device_vector<float>& CDerivativeSolver::GetPressures() const
{
	return m_pressures;
}