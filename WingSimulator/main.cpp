#include "pch.hpp"

constexpr size_t kParticles = 4096 * 128;
constexpr float kParticleRad = 0.002f;

//constexpr size_t kParticles = 2048;
//constexpr float kParticleRad = 0.01f;

void SetupState(wing2d::simulation::ISimulation* simulation)
{
	wing2d::simulation::SimulationState state;
	state.particleRad = kParticleRad;
	state.worldSize.width = (16.0f / 9.0f) * 2.0f;

	state.particles = kParticles;
	state.pos.resize(kParticles);
	state.vel.resize(kParticles);
	state.color.resize(kParticles);

	for (size_t i = 0; i < kParticles; ++i)
	{
		/*auto x = glm::linearRand(-4.0f, -1.25f);
		auto y = glm::linearRand(-1.0f, 1.0f);*/
		auto diskRand = glm::diskRand(0.33f);
		auto rand = diskRand + glm::vec2(-1.0f, -0.25f);
		auto x = rand.x;
		auto y = rand.y;

		state.pos[i] = std::make_tuple(x, y);
		//state.vel[i] = std::make_tuple(1.0f, 0.0f);

		auto normalized = (diskRand + glm::vec2(0.33f)) / 0.66f;
		state.color[i] = std::make_tuple(normalized.x, normalized.y, 1.0f, 1.0f);
	}
	simulation->ResetState(state);
}

int main(int argc, char** argv)
{
	try
	{
		auto renderer = wing2d::rendering::opengl::CreateRenderer(argc, argv);
		auto simulation = wing2d::simulation::cuda::CreateSimulation();

		SetupState(simulation.get());

		float dt = 0.001f;

		renderer->SetOnUpdate([&]()
		{
			renderer->RenderAsync(simulation->GetState());
			dt = simulation->Update(dt);
		});

		//renderer->InitWindowLoop(1920, 1080, true);
		renderer->InitWindowLoop(1280, 720, false);
	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}

	return 0;
}