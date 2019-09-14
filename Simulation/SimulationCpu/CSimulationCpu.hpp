#pragma once
#include "../Simulation.hpp"

#include "CLineSegment.hpp"
#include "CDerivativeSolver.hpp"
#include "OdeSolvers.hpp"

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

				const std::vector<CLineSegment>& GetWalls() const { return m_walls; };
				const std::vector<glm::vec2>& GetWing() const { return m_wingParticles; }
			private:
				wing2d::simulation::SimulationState m_state;

				std::unique_ptr<IOdeSolver> m_odeSolver = {std::make_unique<CForwardEulerSolver>(std::make_unique<CDerivativeSolver>(*this))};

				std::vector<CLineSegment> m_walls;
				std::vector<glm::vec2> m_wingParticles;
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