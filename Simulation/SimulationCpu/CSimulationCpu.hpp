#pragma once
#include "../Simulation.hpp"

#include "CLineSegment.hpp"

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CSimulationCpu : public ISimulation
			{
			public:
				virtual ~CSimulationCpu() override = default;
				virtual void ResetState(const SimulationState& state) override;
				virtual float Update(float dt) override;
				virtual const SimulationState& GetState() const override;
			private:
				wing2d::simulation::SimulationState m_state;
				std::vector<CLineSegment> m_walls;
				std::vector<glm::vec2> m_forces;

				std::vector<glm::vec2> m_wingParticles;

				void BuildWalls();
				void BuildWing();

				static glm::vec2 ComputeForce(const glm::vec2& pos1, const glm::vec2& vel1, const glm::vec2& pos2, const glm::vec2& vel2, float diameter);
				float ComputeMinDeltaTime(float requestedDt) const;
				void ResetForces();
				void ParticleToParticle();
				void ParticleToWing();
				void ParticleToWall();
				void MoveParticles(float dt);
				void ColorParticles();
			};
		}
	}
}