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
				CDerivativeSolver(size_t particles, float radius, const Segments_t& airfoil, const Segments_t& walls);

				virtual void Derive(const OdeState_t& curState, OdeState_t& outDerivative) override;
			private:
				CLineSegmentsStorage m_airfoilStorage;
				CLineSegmentsStorage m_wallsStorage;

				CBoundingBoxesStorage m_airfoilsBoxesStorage;
				CBoundingBoxesStorage m_particlesBoxesStorage;

				const size_t m_particles;
				const float m_particleRad;

				thrust::device_vector<float2> m_forces;

				CMortonTree m_particlesTree;
				CMortonTree m_airfoilTree;

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