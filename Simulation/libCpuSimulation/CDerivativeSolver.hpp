#pragma once

#include <cstdint>
#include <vector>
#include <atomic>
#include <glm/glm.hpp>

#include "../Simulation.hpp"

#include "CMortonTree.hpp"
#include "CLineSegment.hpp"

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CSimulationCpu;

			class CDerivativeSolver : public IDerivativeSolver
			{
			public:
				CDerivativeSolver(const CSimulationCpu& sim);

				virtual void Derive(const OdeState_t& curState, OdeState_t& outDerivative) override;
			private:
				const size_t m_particles;
				const float m_particleRadius;
				const std::vector<CLineSegment>& m_wing;
				const std::vector<CLineSegment>& m_walls;

				CMortonTree m_particlesTree;
				CMortonTree m_wingTree;

				std::vector<glm::vec2> m_forces;
				std::vector<CBoundingBox> m_particlesBoundingBoxes;
				std::vector<std::vector<size_t>> m_potentialCollisionsList;

				void ResetForces();
				void BuildParticlesTree(const OdeState_t& curState);
				void ResolveParticleParticleCollisions(const OdeState_t& curState);
				void ResolveParticleWingCollisions(const OdeState_t& curState);
				void ParticleToWall(const OdeState_t& curState);
				void ApplyGravity();


				static glm::vec2 ComputeForce(const glm::vec2& pos1, const glm::vec2& vel1, const glm::vec2& pos2, const glm::vec2& vel2, float diameter);
			};
		}
	}
}