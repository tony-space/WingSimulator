#include "pch.hpp"

constexpr size_t kParticles = 1024;

int main(int argc, char** argv)
{
	auto simulation = wing2d::cpu::CreateSimulation();

	wing2d::serialization::SimulationState state;

	state.particles.reserve(kParticles);
	std::generate_n(std::back_inserter(state.particles), kParticles, []()
	{
		wing2d::serialization::Particle p;
		p.pos = glm::vec2(glm::linearRand(-1.0f, 1.0f));
		p.vel = glm::vec2(glm::linearRand(-1.0f, 1.0f));

		return p;
	});

	simulation->ResetState(state);
	
	for (float t = 0.0f; t < 1.0f;)
		t += simulation->Update(0.01f);

	simulation->GetState(state);

	return 0;
}