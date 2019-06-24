#include "pch.hpp"

constexpr size_t kParticles = 1024;

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
		if(fscanf(pFile, "%f %f", &result.x, &result.y) != 2)
			throw std::runtime_error("Couldn't read the next vertex of airfoil");
		return result;
	};

	std::deque<glm::vec2> result;
	std::generate_n(std::back_inserter(result), vertices1, vertexParser);
	std::generate_n(std::front_inserter(result), vertices2, vertexParser);
	return std::vector(result.begin(), result.end());
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
		state.wing = LoadAirfoil(argv[1]);

		simulation->ResetState(state);

		renderer->SetOnUpdate([&]()
		{
			simulation->GetState(state);
			renderer->RenderAsync(state);
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