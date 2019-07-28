#include "pch.hpp"

#include "CDerivativeSolver.hpp"
#include "CSimulationCpu.hpp"

using namespace wing2d::simulation::cpu;

static glm::vec2 SpringDamper(const glm::vec2& normal, const glm::vec2& vel1, const glm::vec2& vel2, float springLen)
{
	constexpr float stiffness = 20000.0f;
	constexpr float damp = 50.0f;
	auto v = glm::dot(vel1 - vel2, normal);
	auto force = normal * (springLen * stiffness + v * damp);
	return force;
}
static inline uint64_t ExpandBy1(uint32_t val)
{
	uint64_t result = val;

	result = ((result << 16) | result) & 0x0000FFFF0000FFFF;
	result = ((result << 8) | result) & 0x00FF00FF00FF00FF;
	result = ((result << 4) | result) & 0x0F0F0F0F0F0F0F0F;
	result = ((result << 2) | result) & 0x3333333333333333;
	result = ((result << 1) | result) & 0x5555555555555555;

	return result;
}

static inline uint64_t Morton2D(uint32_t x, uint32_t y)
{
	return (ExpandBy1(x) << 1) | ExpandBy1(y);
}

CDerivativeSolver::CDerivativeSolver(const CSimulationCpu& sim) :
	m_simulation(sim)
{
}

void CDerivativeSolver::operator()(const OdeState_t& state, OdeState_t& derivative)
{
	const auto particles = m_simulation.GetState().particles;
	if (state.size() != particles * 2 || derivative.size() != particles * 2)
		throw std::runtime_error("incorrect state/derivative size");

	m_odeState = &state;
	m_forces.resize(particles);

	ResetForces();
	ParticleToParticle();
	ParticleToWing();
	ParticleToWall();
	ApplyGravity();

	std::copy(state.cbegin() + particles, state.cend(), derivative.begin());
	std::copy(m_forces.cbegin(), m_forces.cend(), derivative.begin() + particles);
}

glm::vec2 CDerivativeSolver::ComputeForce(const glm::vec2& pos1, const glm::vec2& vel1, const glm::vec2& pos2, const glm::vec2& vel2, float diameter)
{
	auto deltaPos = pos2 - pos1;
	auto distSq = glm::dot(deltaPos, deltaPos);
	if (distSq > (diameter * diameter))
		return glm::vec2(0.0f);

	auto dist = glm::sqrt(distSq);
	auto dir = deltaPos / dist;
	auto springLen = diameter - dist;

	auto force = SpringDamper(dir, vel1, vel2, springLen);
	return force;
}

//https://en.wikipedia.org/wiki/Find_first_set#CLZ
//count leading zeros
size_t clz64(uint64_t value)
{
	return 63 - size_t(glm::floor(glm::log(value) / glm::log(2)));
}

void CDerivativeSolver::ResetForces()
{
	std::fill(m_forces.begin(), m_forces.end(), glm::vec2(0.0f));
}

void CDerivativeSolver::TraverseRecursive(const SAbstractNode* subtreeRoot, const CBoundingBox& queryBox, const SLeafNode* self)
{
	if (!queryBox.Overlaps(subtreeRoot->box))
		return;

	if (subtreeRoot->IsLeaf())
	{
		if (self == subtreeRoot)
			return;

		auto leaf = static_cast<const SLeafNode*>(subtreeRoot);
		auto objectIdx = std::get<1>(leaf->object);
		m_potentialCollisionsList.push_back(objectIdx);
	}
	else
	{
		auto internalNode = static_cast<const SInternalNode*>(subtreeRoot);
		TraverseRecursive(internalNode->left, queryBox, self);
		TraverseRecursive(internalNode->right, queryBox, self);
	}
}

void CDerivativeSolver::ParticleToParticle()
{
	const auto& state = m_simulation.GetState();
	const auto particles = state.particles;

	const auto* pos = m_odeState->data();
	const auto* vel = pos + particles;

	const auto diameter = state.particleRad * 2.0f;

	BuildTree(pos, particles);

	for (size_t sortedIdx = 0; sortedIdx < particles; ++sortedIdx)
	{
		m_potentialCollisionsList.clear();

		auto index = std::get<1>(m_sortedMortonCodes[sortedIdx]);
		const auto& self = m_leafNodesPool[sortedIdx];
		TraverseRecursive(m_treeRoot, self.box, &self);

		if (m_potentialCollisionsList.empty())
			continue;

		const auto& p1 = pos[index];
		const auto& v1 = vel[index];

		for (size_t otherObjectIdx : m_potentialCollisionsList)
		{
			const auto& p2 = pos[otherObjectIdx];
			const auto& v2 = vel[otherObjectIdx];

			auto force = ComputeForce(p1, v1, p2, v2, diameter);
			m_forces[index] -= force;
			m_forces[otherObjectIdx] += force;
		}
	}
}

void CDerivativeSolver::ParticleToWing()
{
	const auto& state = m_simulation.GetState();
	const auto& wingParticles = m_simulation.GetWing();

	BuildTree(wingParticles.data(), wingParticles.size());

	const auto particles = state.particles;
	auto pos = *m_odeState | boost::adaptors::sliced(0, particles);
	auto vel = *m_odeState | boost::adaptors::sliced(particles, particles * 2);
	const auto rad = state.particleRad;
	const auto diameter = rad * 2.0f;

	for (size_t i = 0; i < particles; ++i)
	{
		m_potentialCollisionsList.clear();

		const auto &p = pos[i];
		const auto &v = vel[i];
		CBoundingBox box;
		box.AddPoint(glm::vec2(p.x - rad, p.y - rad));
		box.AddPoint(glm::vec2(p.x + rad, p.y + rad));

		TraverseRecursive(m_treeRoot, box);
		if (m_potentialCollisionsList.empty())
			continue;

		for (size_t wingIdx : m_potentialCollisionsList)
		{
			const auto& wp = wingParticles[wingIdx];
			m_forces[i] -= ComputeForce(p, v, wp, glm::vec2(0.0f), diameter);
		}
	}
}

void CDerivativeSolver::ParticleToWall()
{
	const auto& state = m_simulation.GetState();
	const auto& walls = m_simulation.GetWalls();
	const auto particles = state.particles;

	auto pos = *m_odeState | boost::adaptors::sliced(0, particles);
	auto vel = *m_odeState | boost::adaptors::sliced(particles, particles * 2);
	for (size_t i = 0; i < particles; ++i)
	{
		const auto &p = pos[i];
		const auto &v = vel[i];

		for (const auto& w : walls)
		{
			auto d = w.DistanceToLine(p) - state.particleRad;
			if (d < 0.0f && glm::dot(w.normal(), p) < 0.0f)
			{
				auto force = SpringDamper(w.normal(), v, glm::vec2(0.0f), d);
				m_forces[i] -= force;
			}
		}
	}
}

void CDerivativeSolver::ApplyGravity()
{
	for (auto& f : m_forces)
		f.y -= 0.5f;
}

int8_t CDerivativeSolver::Delta(int64_t i, int64_t j) const
{
	int64_t n = int64_t(m_sortedMortonCodes.size());
	if (j < 0 || j > n - 1)
		return -1;

	auto firstCode = std::get<0>(m_sortedMortonCodes[i]);
	auto lastCode = std::get<0>(m_sortedMortonCodes[j]);
	auto bias = int64_t(0);

	if (firstCode == lastCode)
	{
		firstCode = i;
		lastCode = j;
		bias = 64;
	}

	auto commonPrefix = clz64(firstCode ^ lastCode);
	return int8_t(bias + commonPrefix);
}

int64_t CDerivativeSolver::FindSplit(int64_t i, int64_t j) const
{
	auto commonPrefix = Delta(i, j);
	auto d = glm::sign(j - i);

	auto shift = int64_t(0);
	auto step = d * (j - i); //always positive
	do
	{
		step = (step + 1) >> 1; // exponential decrease
		if (Delta(i, i + d * (shift + step)) > commonPrefix)
			shift += step;

	} while (step > 1);

	return i + shift * d + std::min<int64_t>(d, 0);
}

int64_t CDerivativeSolver::FindUpperBound(int64_t i, int64_t d, int64_t dMin) const
{
	auto lMax = 2;
	while (Delta(i, i + lMax * d) > dMin)
		lMax *= 2;

	auto shift = int64_t(0);
	auto step = lMax;
	do
	{
		step = (step + 1) >> 1;
		if (Delta(i, i + (shift + step) * d) > dMin)
			shift += step;
	} while (step > 1);

	return i + shift * d;
}

void CDerivativeSolver::ProcessInternalNode(int64_t i)
{
	auto d = glm::sign(Delta(i, i + 1) - Delta(i, i - 1));
	auto dMin = Delta(i, i - d);

	auto j = FindUpperBound(i, d, dMin);
	auto splitPos = FindSplit(i, j);

	SAbstractNode* left;
	SAbstractNode* right;

	if (glm::min(i, j) == splitPos)
	{
		left = &m_leafNodesPool[splitPos];
	}
	else
	{
		left = &m_internalNodesPool[splitPos];
	}

	if (glm::max(i, j) == splitPos + 1)
	{
		right = &m_leafNodesPool[splitPos + 1];
	}
	else
	{
		right = &m_internalNodesPool[splitPos + 1];
	}

	SInternalNode* self = &m_internalNodesPool[i];

	left->parent = self;
	right->parent = self;
	self->left = left;
	self->right = right;
}

void CDerivativeSolver::BuildTree(const glm::vec2* pos, size_t particlesCount)
{
	CBoundingBox particlesBox;

	particlesBox.AddPoints(pos, pos + particlesCount);
	m_sortedMortonCodes.resize(particlesCount);
	for (size_t i = 0; i < particlesCount; ++i)
	{
		const auto& p = pos[i];
		auto normalized = (p - particlesBox.min()) / particlesBox.size();
		assert(normalized.x >= 0.0f);
		assert(normalized.y >= 0.0f);
		assert(normalized.x <= 1.0f);
		assert(normalized.y <= 1.0f);

		uint32_t x = uint32_t(glm::round(normalized.x * double(0xFFFFFFFF)));
		uint32_t y = uint32_t(glm::round(normalized.y * double(0xFFFFFFFF)));
		uint64_t morton = Morton2D(x, y);

		m_sortedMortonCodes[i] = std::make_tuple(morton, i);
	}

	std::sort(m_sortedMortonCodes.begin(), m_sortedMortonCodes.end(), [](const auto& t1, const auto& t2)
	{
		return std::get<0>(t1) < std::get<0>(t2);
	});

	const int64_t internalCount = particlesCount - 1;

	m_internalNodesPool.clear();
	m_leafNodesPool.clear();

	m_internalNodesPool.resize(internalCount);
	m_leafNodesPool.reserve(particlesCount);
	for (const auto& obj : m_sortedMortonCodes)
		m_leafNodesPool.emplace_back(obj);

	for (int64_t i = 0; i < internalCount; ++i)
		ProcessInternalNode(i);

	if (particlesCount > 1)
		m_treeRoot = &m_internalNodesPool[0];
	else
		m_treeRoot = &m_leafNodesPool[0];

	ConstructBoundingBoxes(pos, m_simulation.GetState().particleRad);
}

void CDerivativeSolver::ConstructBoundingBoxes(const glm::vec2* pos, float rad)
{
	int64_t leafs = m_leafNodesPool.size();

	for (int64_t i = 0; i < leafs; ++i)
	{
		auto p = pos[std::get<1>(m_leafNodesPool[i].object)];
		auto& box = m_leafNodesPool[i].box;
		box.AddPoint(glm::vec2(p.x + rad, p.y + rad));
		box.AddPoint(glm::vec2(p.x - rad, p.y - rad));
	}

	std::function<void(SAbstractNode* subTree)> recursive;

	recursive = [&](SAbstractNode* subTree)
	{
		if (subTree->IsLeaf())
			return;

		SInternalNode* self = static_cast<SInternalNode*>(subTree);
		if (self->left)
		{
			recursive(self->left);
			self->box.AddBox(self->left->box);
		}
		if (self->right)
		{
			recursive(self->right);
			self->box.AddBox(self->right->box);
		}
	};

	recursive(m_treeRoot);
}

bool CDerivativeSolver::SLeafNode::IsLeaf() const
{
	return true;
}

bool CDerivativeSolver::SInternalNode::IsLeaf() const
{
	return false;
}
