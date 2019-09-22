#pragma once

#include <vector>
#include <memory>
#include <cstdint>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

#ifndef SIMULATION_IMPL
#define SIMULATION_API __declspec(dllimport)
#else
#define SIMULATION_API __declspec(dllexport)
#endif // !SIMULATION_IMPL


namespace wing2d
{
	namespace simulation
	{
		typedef std::vector<glm::vec2> OdeState_t;

		struct SimulationState
		{
			struct
			{
				float width = 2.0f;
				float height = 2.0f;
			} worldSize;

			float particleRad = 0.01f;
			size_t particles = 0;

			std::vector<glm::vec2> pos;
			std::vector<glm::vec2> vel;
			std::vector<glm::vec4> color;
			std::vector<glm::vec2> airfoil;

			bool IsValid() const
			{
				return pos.size() == particles && vel.size() == particles && color.size() == particles;
			}
		};

		struct ISimulation
		{
			virtual void ResetState(const SimulationState& state) = 0;
			virtual float Update(float dt) = 0;
			virtual const SimulationState& GetState() const = 0;
			virtual ~ISimulation() = default;
		};

		struct IDerivativeSolver
		{
			virtual void Derive(const OdeState_t& curState, OdeState_t& outDerivative) = 0;
			virtual ~IDerivativeSolver() = default;
		};

		struct IOdeSolver
		{
			virtual void NextState(const OdeState_t& curState, const float dt, OdeState_t& outNextState) = 0;
			virtual ~IOdeSolver() = default;
		};

		namespace cpu
		{
			std::unique_ptr<ISimulation> SIMULATION_API CreateSimulation();
		}
	}
}