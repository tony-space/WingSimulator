#include "pch.hpp"

constexpr size_t kParticles = 1024;

int main(int argc, char** argv)
{
	auto renderer = wing2d::rendering::opengl::CreateRenderer(argc, argv);
	auto simulation = wing2d::simulation::cpu::CreateSimulation();

	wing2d::simulation::serialization::SimulationState state;

	state.particles.reserve(kParticles);
	std::generate_n(std::back_inserter(state.particles), kParticles, []()
	{
		wing2d::simulation::serialization::Particle p;
		p.pos = glm::vec2(glm::linearRand(-1.0f, 1.0f));
		p.vel = glm::vec2(glm::linearRand(-1.0f, 1.0f));

		return p;
	});

	simulation->ResetState(state);

	renderer->SetOnUpdate([&]()
	{
		simulation->GetState(state);
		renderer->RenderAsync(state);
		float t = simulation->Update(0.01f);
		renderer->Synchronize();
	});

	renderer->InitWindowLoop(800, 800, false);

	return 0;
}