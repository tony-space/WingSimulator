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

					const float2* __restrict__ pos;
					const float2* __restrict__ vel;
					float2* __restrict__ force;
					float* __restrict__ pressure;
				};

				CDerivativeSolver(size_t particles, float radius, const Segments_t& airfoil, const Segments_t& walls);

				virtual void Derive(const OdeState_t& curState, OdeState_t& outDerivative) override;

				const thrust::device_vector<float>& GetPressures() const;
			private:
				CLineSegmentsStorage m_airfoilStorage;
				CLineSegmentsStorage m_wallsStorage;

				CBoundingBoxesStorage m_airfoilsBoxesStorage;
				CBoundingBoxesStorage m_particlesBoxesStorage;
				CBoundingBoxesStorage m_particlesExtendedBoxesStorage;

				const size_t m_particles;
				const float m_particleRad;

				thrust::device_vector<float2> m_forces;
				thrust::device_vector<float> m_pressures;

				CMortonTree m_particlesTree;
				CMortonTree m_airfoilTree;

				SIntermediateSimState GetSimState(const OdeState_t& curState);

				void ResetForces();
				void BuildParticlesTree(const OdeState_t& curState);
				void ResolveParticleParticleCollisions(const OdeState_t& curState);
				void ResolveParticleWingCollisions(const OdeState_t& curState);
				void ParticleToWall(const OdeState_t& curState);
				void ApplyGravity();
				void BuildDerivative(const OdeState_t& curState, OdeState_t& outDerivative) const;
			};
		}
	}
}