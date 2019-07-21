#pragma once
#include "../Simulation.hpp"

#include "CLineSegment.hpp"
#include "CDerivativeSolver.hpp"
#include "COdeSolver.hpp"

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CSimulationCpu : public ISimulation
			{
			public:
				CSimulationCpu();

				virtual ~CSimulationCpu() override = default;
				virtual void ResetState(const SimulationState& state) override;
				virtual float Update(float dt) override;
				virtual const SimulationState& GetState() const override;

				const std::vector<CLineSegment>& GetWalls() const { return m_walls; };
				const std::vector<glm::vec2>& GetWing() const { return m_wingParticles; }
			private:
				wing2d::simulation::SimulationState m_state;
				std::vector<CLineSegment> m_walls;
				std::vector<glm::vec2> m_wingParticles;
				CDerivativeSolver m_derivativeSolver;
				COdeSolver m_odeSolver;
				std::vector<glm::vec2> m_odeState;
				std::vector<glm::vec2> m_odeNextStateRude;
				std::vector<glm::vec2> m_odeNextStatePrecise1;
				std::vector<glm::vec2> m_odeNextStatePrecise2;

				void BuildWalls();
				void BuildWing();

				float ComputeMinDeltaTime(float requestedDt) const;
				void ColorParticles(float dt);
			};
		}
	}
}