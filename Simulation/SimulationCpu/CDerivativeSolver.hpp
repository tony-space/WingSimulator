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

				CMortonTree m_particlesTree;
				std::vector<glm::vec2> m_allParticles;
				std::vector<glm::vec2> m_forces;
				std::vector<std::vector<size_t>> m_potentialCollisionsList;

				void ResetForces();
				void BuildTree();
				void ResolveCollisions();
				void ParticleToWall();
				void ApplyGravity();

				static glm::vec2 ComputeForce(const glm::vec2& pos1, const glm::vec2& vel1, const glm::vec2& pos2, const glm::vec2& vel2, float diameter);
			};
		}
	}
}