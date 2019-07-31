#pragma once

#include <cstdint>
#include <vector>
#include <atomic>
#include <glm/glm.hpp>

#include "CMortonTree.hpp"

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CSimulationCpu;

			class CDerivativeSolver
			{
			public:
				typedef std::vector<glm::vec2> OdeState_t;

				CDerivativeSolver(const CSimulationCpu& sim);
				void operator() (const OdeState_t& odeState, OdeState_t& odeDerivative);
			private:
				const CSimulationCpu& m_simulation;
				const OdeState_t* m_odeState = nullptr;

				CMortonTree m_p2pTree;
				CMortonTree m_p2wTree;

				std::vector<glm::vec2> m_p2pForces;
				std::vector<glm::vec2> m_p2wForces;
				std::vector<glm::vec2> m_combinedForces;

				std::vector<std::vector<size_t>> m_potentialCollisionsListP2p;
				std::vector<std::vector<size_t>> m_potentialCollisionsListP2w;

				void ResetForces();
				void ParticleToParticle();
				void ParticleToWing();
				void ParticleToWall();
				void CombineForces();

				static glm::vec2 ComputeForce(const glm::vec2& pos1, const glm::vec2& vel1, const glm::vec2& pos2, const glm::vec2& vel2, float diameter);
			};
		}
	}
}