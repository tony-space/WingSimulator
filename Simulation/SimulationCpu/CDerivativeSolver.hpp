#pragma once

#include <vector>
#include <glm/glm.hpp>

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
				std::vector<glm::vec2> m_forces;

				static glm::vec2 ComputeForce(const glm::vec2& pos1, const glm::vec2& vel1, const glm::vec2& pos2, const glm::vec2& vel2, float diameter);

				void ResetForces();
				void ParticleToParticle(const OdeState_t& odeState);
				void ParticleToWing(const OdeState_t& odeState);
				void ParticleToWall(const OdeState_t& odeState);
				void ApplyGravity();
			};
		}
	}
}