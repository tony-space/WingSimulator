#include "pch.hpp"

constexpr size_t kParticles = 8192;
constexpr float kParticleRad = 0.005f;

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

std::vector<glm::vec2> LoadAirfoil(const char* path)
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

	std::deque<glm::vec2> deque;
	std::vector<glm::vec2> airfoil;
	std::generate_n(std::back_inserter(deque), vertices1, vertexParser);
	std::generate_n(std::front_inserter(deque), vertices2, vertexParser);

	auto range = deque | boost::adaptors::transformed([](const glm::vec2& v) { return std::make_tuple(v.x, v.y); });

	std::set uniquePoints(range.begin(), range.end());
	airfoil.reserve(uniquePoints.size());
	for (const glm::vec2& v : deque)
	{
		auto it = uniquePoints.find(std::make_tuple(v.x, v.y));
		if (it != uniquePoints.end())
		{
			airfoil.emplace_back(v);
			uniquePoints.erase(it);
		}
	}
	return airfoil;
}

void SetupState(wing2d::simulation::ISimulation* simulation, std::vector<glm::vec2>&& airfoil)
{
	wing2d::simulation::serialization::SimulationState state;
	state.particleRad = kParticleRad;
	state.worldSize.width = (4.0f / 3.0f) * 2.0f;

	state.particles.reserve(kParticles);
	std::generate_n(std::back_inserter(state.particles), kParticles, [&]()
	{
		auto p = wing2d::simulation::serialization::Particle();
		//p.pos = glm::linearRand(glm::vec2(-1.0f + kParticleRad), glm::vec2(0.0f - kParticleRad, 1.0f - kParticleRad));
		p.pos.x = glm::linearRand(state.worldSize.width / -2.0f, state.worldSize.width / 2.0f);
		p.pos.y = glm::linearRand(0.25f, 1.0f);

		//p.vel = glm::linearRand(glm::vec2(-0.3f), glm::vec2(0.3f));
		
		return p;
	});

	std::transform(airfoil.cbegin(), airfoil.cend(), airfoil.begin(), [](const auto& a)
	{
		return a - glm::vec2(0.5f, 0.5f);
	});

	state.airfoil = std::move(airfoil);

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
			float t = simulation->Update(0.0005f);
		});

		//renderer->InitWindowLoop(1920, 1080, true);
		renderer->InitWindowLoop(1024, 768, false);
	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}

	return 0;
}