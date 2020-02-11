#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "CudaLaunchHelpers.cuh"
#include "CDerivativeSolver.cuh"

#include "CMortonTreeTraversal.cuh"

using namespace wing2d::simulation::cuda;

static __device__ float2 SpringDamper(const float2& normal, const float2& vel1, const float2& vel2, float springLen)
{
	constexpr float stiffness = 10000.0f;
	constexpr float damp = 10.0f;
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

static __global__ void ParticleToWallKernel(CDerivativeSolver::SIntermediateSimState state, SLineSegmentsSOA walls)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= state.particles)
		return;

	const auto pos = state.pos[threadId];
	const auto vel = state.vel[threadId];
	
	auto force = make_float2(0.0f);

	for (size_t i = 0; i < walls.lineSegments; ++i)
	{
		const auto d = walls.DistanceToLine(i, pos) - state.particleRad;
		if (d < 0.0f)
		{
			force += SpringDamper(walls.normal[i], vel, make_float2(0.0f), d);
		}
	}

	state.force[threadId] += force;
}

static __global__ void AddGravityKernel(float2* forces, unsigned n)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= n)
		return;

	forces[threadId].y -= 0.5f;
}

static __global__ void BuildAirfoilBoxesKernel(SLineSegmentsSOA airfoil, SBoundingBoxesAoS boxes)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= airfoil.lineSegments)
		return;

	const auto f = airfoil.first[threadId];
	const auto s = airfoil.second[threadId];

	const auto minCorner = fminf(f, s);
	const auto maxCorner = fmaxf(f, s);

	boxes.boxes[threadId] = { minCorner, maxCorner };
}

static __global__ void ReorderAirfoilKernel(const SLineSegmentsSOA oldAirfoil, SLineSegmentsSOA newAirfoil, const TIndex* __restrict__ oldIndices)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= oldAirfoil.lineSegments)
		return;

	auto oldIdx = oldIndices[threadId];

	newAirfoil.first[threadId] = oldAirfoil.first[oldIdx];
	newAirfoil.second[threadId] = oldAirfoil.second[oldIdx];
	newAirfoil.ray[threadId] = oldAirfoil.ray[oldIdx];
	newAirfoil.normal[threadId] = oldAirfoil.normal[oldIdx];
	newAirfoil.length[threadId] = oldAirfoil.length[oldIdx];
}

static __global__ void BuildParticlesBoundingBoxesKernel(const float2* __restrict__ particlePos, float particleRad, SBoundingBoxesAoS boundingBoxes)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= boundingBoxes.count)
		return;

	const auto pos = particlePos[threadId];
	auto minCorner = make_float2(pos.x - particleRad, pos.y - particleRad);
	auto maxCorner = make_float2(pos.x + particleRad, pos.y + particleRad);

	boundingBoxes.boxes[threadId] = { minCorner, maxCorner };
}

static __global__ void ReorderParticlesKernel(const float2* __restrict__ originalStateVector, const TIndex* __restrict__ oldIndices, CDerivativeSolver::SIntermediateSimState simState)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= simState.particles)
		return;

	auto oldPos = originalStateVector;
	auto oldVel = oldPos + simState.particles;

	auto oldIdx = oldIndices[threadId];
	simState.pos[threadId] = oldPos[oldIdx];
	simState.vel[threadId] = oldVel[oldIdx];
}

static __global__ void ResetParticlesStateKernel(CDerivativeSolver::SIntermediateSimState simState)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= simState.particles)
		return;

	simState.force[threadId] = make_float2(0.0f);
	simState.pressure[threadId] = 0.0f;
}

struct SParticleParticleCollisionSolver
{
	struct SDeviceSideSolver
	{
		CDerivativeSolver::SIntermediateSimState& simState;
		TIndex curIdx;
		float2 pos1;
		float2 vel1;
		float2 totalForce;
		float totalPressure;

		__device__ SDeviceSideSolver(CDerivativeSolver::SIntermediateSimState& state) : simState(state)
		{

		}

		__device__ void OnPreTraversal(TIndex curLeafIdx)
		{
			curIdx = curLeafIdx;
			pos1 = simState.pos[curLeafIdx];
			vel1 = simState.vel[curLeafIdx];
			totalForce = make_float2(0.0f);
			totalPressure = 0.0f;
		}

		__device__ void OnCollisionDetected(TIndex anotherLeafIdx)
		{
			const auto pos2 = simState.pos[anotherLeafIdx];
			const auto deltaPos = pos2 - pos1;
			const auto distanceSq = dot(deltaPos, deltaPos);
			if (distanceSq > simState.diameterSq || distanceSq < 1e-8f)
				return;

			const auto vel2 = simState.vel[anotherLeafIdx];

			auto dist = sqrtf(distanceSq);
			auto dir = deltaPos / dist;
			auto springLen = simState.diameter - dist;

			auto force = SpringDamper(dir, vel1, vel2, springLen);
			auto pressure = length(force);
			totalForce += force;
			totalPressure += pressure;

			atomicAdd(&simState.force[anotherLeafIdx].x, -force.x);
			atomicAdd(&simState.force[anotherLeafIdx].y, -force.y);
			atomicAdd(&simState.pressure[anotherLeafIdx], pressure);
		}

		__device__ void OnPostTraversal()
		{
			atomicAdd(&simState.force[curIdx].x, totalForce.x);
			atomicAdd(&simState.force[curIdx].y, totalForce.y);
			atomicAdd(&simState.pressure[curIdx], totalPressure);
		}
	};

	CDerivativeSolver::SIntermediateSimState simState;

	SParticleParticleCollisionSolver(const CDerivativeSolver::SIntermediateSimState& state) : simState(state)
	{
	}

	__device__ SDeviceSideSolver Create()
	{
		return SDeviceSideSolver(simState);
	}
};

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
		const auto wingSegmentIdx = potentialCollisions.internalIndices[collisionIdx * simState.particles + threadId];
		if (wingSegmentIdx == TIndex(-1))
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

static __global__ void InverseForcesOrderKernel(const CDerivativeSolver::SIntermediateSimState simState, const TIndex* __restrict__ oldIndices, float2* __restrict__ unorderedForces)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= simState.particles)
		return;
	auto oldIdx = oldIndices[threadId];
	unorderedForces[oldIdx] = simState.force[threadId];
}

//
//
//
CDerivativeSolver::CDerivativeSolver(size_t particles, float radius, const Segments_t& airfoil, const Segments_t& walls) :
	m_airfoilStorage(airfoil.size()),
	m_wallsStorage(walls),
	m_particlesBoxes(particles),
	m_particlesExtendedBoxes(particles),
	m_particles(particles, radius)
{
	auto tempAirfoilStorage = CLineSegmentsStorage(airfoil);
	auto tempAirfoilBoxes = thrust::device_vector<SBoundingBox>(airfoil.size());

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(airfoil.size(), kBlockSize));
	BuildAirfoilBoxesKernel <<<gridDim, blockDim >>> (tempAirfoilStorage.get(), SBoundingBoxesAoS::Create(tempAirfoilBoxes));
	CudaCheckError();
	m_airfoilTree.Build(tempAirfoilBoxes);

	ReorderAirfoilKernel <<<gridDim, blockDim >>> (tempAirfoilStorage.get(), m_airfoilStorage.get(), m_airfoilTree.GetSortedIndices().data().get());
	CudaCheckError();
}

void CDerivativeSolver::Derive(const OdeState_t& curState, OdeState_t& outDerivative)
{
	BuildParticlesTree(curState);
	ReorderParticles(curState);
	ResetParticlesState();
	ResolveParticleParticleCollisions();
	//ResolveParticleWingCollisions();
	ParticleToWall();
	ApplyGravity();
	BuildDerivative(curState, outDerivative);
}

CDerivativeSolver::SIntermediateSimState CDerivativeSolver::SParticlesState::GetSimState()
{
	return
	{
		count,
		radius,
		radius * 2.0f,
		radius * radius * 4.0f,

		reorderedPositions.data().get(),
		reorderedVelocities.data().get(),
		forces.data().get(),
		pressures.data().get(),
	};
}

void CDerivativeSolver::BuildParticlesTree(const OdeState_t& curState)
{
	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(m_particles.count, kBlockSize));

	BuildParticlesBoundingBoxesKernel <<<gridDim, blockDim >>> (curState.data().get(), m_particles.radius, SBoundingBoxesAoS::Create(m_particlesBoxes));

	CudaCheckError();

	m_particlesTree.Build(m_particlesBoxes);
}

void CDerivativeSolver::ReorderParticles(const OdeState_t& curState)
{
	auto oldIndices = m_particlesTree.GetSortedIndices().data().get();

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(m_particles.count, kBlockSize));

	ReorderParticlesKernel <<<gridDim, blockDim >>> (curState.data().get(), oldIndices, m_particles.GetSimState());
	BuildParticlesBoundingBoxesKernel <<<gridDim, blockDim >>> (m_particles.reorderedPositions.data().get(), m_particles.radius * 4.0f, SBoundingBoxesAoS::Create(m_particlesExtendedBoxes));

	CudaCheckError();
}

void CDerivativeSolver::ResetParticlesState()
{
	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(m_particles.count, kBlockSize));

	ResetParticlesStateKernel <<<gridDim, blockDim >>> (m_particles.GetSimState());
}

void CDerivativeSolver::ResolveParticleParticleCollisions()
{
	m_particlesTree.TraverseReflexive<SParticleParticleCollisionSolver, 24>(SParticleParticleCollisionSolver(m_particles.GetSimState()));
	CudaCheckError();
}

void CDerivativeSolver::ResolveParticleWingCollisions()
{
	const auto collisionsResult = m_airfoilTree.Traverse(m_particlesExtendedBoxes);

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(m_particles.count, kBlockSize));

	auto simState = m_particles.GetSimState();
	ResolveParticleWingCollisionsKernel <<<gridDim, blockDim >>> (collisionsResult, simState, m_airfoilStorage.get());

	CudaCheckError();
}

void CDerivativeSolver::ParticleToWall()
{
	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(m_particles.count, kBlockSize));

	ParticleToWallKernel <<<gridDim, blockDim >>> (m_particles.GetSimState(), m_wallsStorage.get());
	CudaCheckError();
}

void CDerivativeSolver::ApplyGravity()
{
	auto elements = unsigned(m_particles.count);
	dim3 blockDim(kBlockSize);
	dim3 gridDim((elements - 1) / blockDim.x + 1);

	AddGravityKernel <<<gridDim, blockDim >>> (m_particles.forces.data().get(), elements);
	CudaCheckError();
}

void CDerivativeSolver::BuildDerivative(const OdeState_t& curState, OdeState_t& outDerivative)
{
	const float2* d_velocities = curState.data().get() + m_particles.count;
	
	//TODO: reorder forces back
	const float2* d_forces = m_particles.forces.data().get();
	float2* d_derivative = outDerivative.data().get();

	const size_t dataBlockSize = m_particles.count * sizeof(float2);

	CudaSafeCall(cudaMemcpyAsync(d_derivative, d_velocities, dataBlockSize, cudaMemcpyKind::cudaMemcpyDeviceToDevice));

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(m_particles.count, kBlockSize));
	InverseForcesOrderKernel <<<gridDim, blockDim >>> (
		m_particles.GetSimState(),
		m_particlesTree.GetSortedIndices().data().get(),
		d_derivative + m_particles.count
		);

	CudaCheckError();
}

const thrust::device_vector<float>& CDerivativeSolver::GetPressures() const
{
	return m_particles.pressures;
}

const thrust::device_vector<TIndex>& CDerivativeSolver::GetParticlesIndices() const
{
	return m_particlesTree.GetSortedIndices();
}