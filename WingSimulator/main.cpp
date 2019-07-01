#include "pch.hpp"

constexpr size_t kParticles = 64;

class CSmartFile
{
public:
	CSmartFile(const char* path)
	{
		m_pFile = fopen(path, "rb");
		if (m_pFile == nullptr)
			throw std::invalid_argument("could not open the file");
	}

	~CSmartFile()
	{
		fclose(m_pFile);
	}

	operator FILE* () const
	{
		return m_pFile;
	}
private:
	FILE* m_pFile;
};

wing2d::simulation::serialization::SimulationState::SWing LoadAirfoil(const char* path)
{
	CSmartFile pFile(path);

	{
		char name[0x1000];
		fgets(name, sizeof(name), pFile);
		printf("Loading %s\n", name);
	}

	float fVertices1, fVertices2;
	if (fscanf(pFile, "%f %f", &fVertices1, &fVertices2) != 2)
		throw std::runtime_error("Couldn't read airfoil header");

	size_t vertices1 = size_t(fVertices1);
	size_t vertices2 = size_t(fVertices2);

	auto vertexParser = [&]()
	{
		glm::vec2 result;
		if (fscanf(pFile, "%f %f", &result.x, &result.y) != 2)
			throw std::runtime_error("Couldn't read the next vertex of airfoil");
		return result;
	};

	using namespace wing2d::simulation::serialization;
	SimulationState::SWing wing;

	{
		std::deque<glm::vec2> deque;
		std::generate_n(std::back_inserter(deque), vertices1, vertexParser);
		std::generate_n(std::front_inserter(deque), vertices2, vertexParser);
		wing.airfoil.reserve(deque.size());
		std::copy(deque.cbegin(), deque.cend(), std::back_inserter(wing.airfoil));
	}

	//std::transform(wing.airfoil.cbegin(), wing.airfoil.cend(), wing.airfoil.begin(), [](const auto& v) {return v * 3.0f - glm::vec2(1.25f, 0.0f); });

	/*wing.airfoil = std::vector<glm::vec2>({
			glm::vec2(-0.5f, -0.5f),
			glm::vec2(0.5f, -0.5f),
			glm::vec2(0.5f, 0.5f),
			glm::vec2(-0.5f, 0.5f) });*/

	{
		std::vector<std::pair<float, float>> curve;
		boost::push_back(curve, wing.airfoil | boost::adaptors::transformed([](const auto& v) {return std::make_pair(v.x, v.y); }));

		std::vector<decltype(curve)> curves = { curve };
		auto indices = mapbox::earcut<uint16_t>(curves);
		wing.triangles.reserve(indices.size() / 3);
		boost::push_back(wing.triangles, indices | boost::adaptors::strided(3) | boost::adaptors::transformed([](const auto& index)
		{
			using namespace wing2d::simulation::serialization;
			SimulationState::SWing::STriangle t;
			t.i1 = index;
			t.i2 = *(&index + 1);
			t.i3 = *(&index + 2);
			return t;
		}));
	}
	return wing;
}

void SetupState(wing2d::simulation::ISimulation* simulation, wing2d::simulation::serialization::SimulationState::SWing&& wing)
{
	wing2d::simulation::serialization::SimulationState state;

	state.particles.reserve(kParticles);
	std::generate_n(std::back_inserter(state.particles), kParticles, []()
	{
		wing2d::simulation::serialization::Particle p;
		p.pos = glm::linearRand(glm::vec2(-1.0f), glm::vec2(1.0f));
		p.vel = glm::linearRand(glm::vec2(-1.0f), glm::vec2(1.0f));

		return p;
	});
	state.worldSize.width = (4.0f / 3.0f) * 2.0f;
	state.wing = std::move(wing);

	simulation->ResetState(state);
}

int main(int argc, char** argv)
{
	try
	{
		if (argc == 1)
		{
			printf("Specify path to airfoil data\n");
			return -1;
		}
		auto renderer = wing2d::rendering::opengl::CreateRenderer(argc, argv);
		auto simulation = wing2d::simulation::cpu::CreateSimulation();

		SetupState(simulation.get(), LoadAirfoil(argv[1]));

		renderer->SetOnUpdate([&]()
		{
			renderer->RenderAsync(simulation->GetState());
			float t = simulation->Update(0.0001f);
		});

		renderer->InitWindowLoop(800, 600, false);
	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}

	return 0;
}