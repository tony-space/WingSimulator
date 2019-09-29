#pragma once

#include <vector>
#include <memory>
#include <cstdint>
#include <tuple>

#ifndef SIMULATION_IMPL
#define SIMULATION_API __declspec(dllimport)
#else
#define SIMULATION_API __declspec(dllexport)
#endif // !SIMULATION_IMPL


namespace wing2d
{
	namespace simulation
	{
		struct SimulationState
		{
			typedef std::tuple<float, float> vec2;
			typedef std::tuple<float, float, float, float> vec4;
			struct
			{
				float width = 2.0f;
				float height = 2.0f;
			} worldSize;

			float particleRad = 0.01f;
			size_t particles = 0;

			std::vector<vec2> pos;
			std::vector<vec2> vel;
			std::vector<vec4> color;
			std::vector<vec2> airfoil;

			bool IsValid() const
			{
				return pos.size() == particles && vel.size() == particles && color.size() == particles;
			}
		};

		struct ISimulation
		{
			virtual void ResetState(const SimulationState& state) = 0;
			virtual float Update(float dt) = 0;
			virtual const SimulationState& GetState() = 0;
			virtual ~ISimulation() = default;
		};

		namespace cpu
		{
			std::unique_ptr<ISimulation> SIMULATION_API CreateSimulation();
		}

		namespace opencl
		{
			std::unique_ptr<ISimulation> SIMULATION_API CreateSimulation();
		}

		namespace cuda
		{
			std::unique_ptr<ISimulation> SIMULATION_API CreateSimulation();
		}
	}
}