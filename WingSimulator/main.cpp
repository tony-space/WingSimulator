#include "pch.hpp"

constexpr size_t kParticles = 4096 * 32;
constexpr float kParticleRad = 0.005f;

//constexpr size_t kParticles = 2048;
//constexpr float kParticleRad = 0.01f;

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

		result *= 0.5f;
		return result;
	};

	std::deque<glm::vec2> deque;
	std::vector<glm::vec2> airfoil;
	std::generate_n(std::back_inserter(deque), vertices1, vertexParser);
	std::generate_n(std::front_inserter(deque), vertices2, vertexParser);

	std::set<std::tuple<float, float>> uniquePoints;
	std::transform(deque.begin(), deque.end(), std::inserter(uniquePoints, uniquePoints.end()), [](const glm::vec2& v) { return std::make_tuple(v.x, v.y); });

	airfoil.reserve(uniquePoints.size());
	std::copy_if(deque.cbegin(), deque.cend(), std::back_inserter(airfoil), [&](const glm::vec2& v)
		{
			auto it = uniquePoints.find(std::make_tuple(v.x, v.y));
			auto isUnique = it != uniquePoints.end();
			if (isUnique)
				uniquePoints.erase(it);
			return isUnique;
		});

	return airfoil;
}

void SetupState(wing2d::simulation::ISimulation* simulation, std::vector<glm::vec2>&& airfoil)
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
		//state.pos[i].x = glm::linearRand(state.worldSize.width / -2.0f, state.worldSize.width / 2.0f);
		//state.pos[i].y = glm::linearRand(0.25f, 1.0f);
		state.pos[i].x = glm::linearRand(-4.0f, -1.25f);
		state.pos[i].y = glm::linearRand(-1.0f, 1.0f);

		state.vel[i] = glm::vec2(1.0f, 0.0f);
	}

	glm::mat3 modelMat = glm::identity<glm::mat3>();
	modelMat = glm::translate(modelMat, glm::vec2(-1.2f, 0.0f));
	modelMat = glm::rotate(modelMat, -10.0f / 180.0f * float(M_PI));

	std::transform(airfoil.cbegin(), airfoil.cend(), airfoil.begin(), [&](const auto& a) -> glm::vec2
		{
			return (modelMat * glm::vec3(a, 1.0f)).xy;
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