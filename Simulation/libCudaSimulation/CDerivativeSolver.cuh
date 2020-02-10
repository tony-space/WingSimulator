#pragma once

#include <vector>
#include <tuple>

#include <helper_math.h>

#include "CudaInterfaces.cuh"
#include "LineSegment.cuh"
#include "BoundingBox.cuh"
#include "CMortonTree.cuh"

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			class CDerivativeSolver : public IDerivativeSolver
			{
			public:
				struct SIntermediateSimState
				{
					const size_t particles;
					const float particleRad;
					const float diameter;
					const float diameterSq;

					float2* __restrict__ pos;
					float2* __restrict__ vel;
					float2* __restrict__ force;
					float* __restrict__ pressure;
				};

				CDerivativeSolver(size_t particles, float radius, const Segments_t& airfoil, const Segments_t& walls);

				virtual void Derive(const OdeState_t& curState, OdeState_t& outDerivative) override;

				const thrust::device_vector<float>& GetPressures() const;
				const thrust::device_vector<TIndex>& GetParticlesIndices() const;
			private:
				CLineSegmentsStorage m_airfoilStorage;
				CLineSegmentsStorage m_wallsStorage;

				thrust::device_vector<SBoundingBox> m_particlesBoxes;
				thrust::device_vector<SBoundingBox> m_particlesExtendedBoxes;

				struct SParticlesState
				{
					const size_t count;
					const float radius;

					thrust::device_vector<float2> reorderedPositions;
					thrust::device_vector<float2> reorderedVelocities;
					thrust::device_vector<float2> forces;
					thrust::device_vector<float> pressures;

					SParticlesState(size_t _count, float _radius) :
						count(_count),
						radius(_radius),
						reorderedPositions(_count),
						reorderedVelocities(_count),
						forces(_count),
						pressures(_count)
					{
					}

					SIntermediateSimState GetSimState();

				} m_particles;

				CMortonTree m_particlesTree;
				CMortonTree m_airfoilTree;


				void BuildParticlesTree(const OdeState_t& curState);
				void ReorderParticles(const OdeState_t& curState);
				void ResetParticlesState();
				void ResolveParticleParticleCollisions();
				void ResolveParticleWingCollisions();
				void ParticleToWall();
				void ApplyGravity();
				void BuildDerivative(const OdeState_t& curState, OdeState_t& outDerivative);
			};
		}
	}
}