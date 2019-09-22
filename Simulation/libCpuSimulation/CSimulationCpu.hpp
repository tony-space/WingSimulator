#pragma once
#include "../Simulation.hpp"

#include "CLineSegment.hpp"
#include "CpuInterfaces.h"
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

				void ColorParticles(float dt);

				virtual const SimulationState& GetState() const override;

				size_t GetParticlesCount() const;
				float GetParticleRadius() const;

				const std::vector<CLineSegment>& GetWalls() const;
				const std::vector<CLineSegment>& GetWing() const;
			private:
				mutable wing2d::simulation::SimulationState m_state;

				std::unique_ptr<IOdeSolver> m_odeSolver = nullptr;

				std::vector<CLineSegment> m_walls;
				std::vector<CLineSegment> m_wing;

				OdeState_t m_curOdeState;
				OdeState_t m_nextOdeState;

				void BuildWalls();
				void BuildWing();

				float ComputeMinDeltaTime(float requestedDt) const;
			};
		}
	}
}