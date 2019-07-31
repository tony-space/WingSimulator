#include "pch.hpp"

#include "CDerivativeSolver.hpp"
#include "CSimulationCpu.hpp"

#include "intrin.h"

using namespace wing2d::simulation::cpu;

static glm::vec2 SpringDamper(const glm::vec2& normal, const glm::vec2& vel1, const glm::vec2& vel2, float springLen)
{
	constexpr float stiffness = 10000.0f;
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

#ifdef _WIN64
#define clz64 __lzcnt64
#elif _WIN32
size_t clz64(uint64_t value)
{
	uint32_t lowDword = static_cast<uint32_t>(value);
	uint32_t highDword = static_cast<uint32_t>(value >> 32);
	
	unsigned long pos;
	if (_BitScanReverse(&pos, highDword))
	{
		return 31 - pos;
	}
	else if(_BitScanReverse(&pos, lowDword))
	{
		return 63 - pos;
	}

	return 64;
}
#else
//https://en.wikipedia.org/wiki/Find_first_set#CLZ
//count leading zeros
size_t clz64(uint64_t value)
{
	return 63 - size_t(glm::floor(glm::log(value) / glm::log(2)));
}
#endif

void CDerivativeSolver::ResetForces()
{
	std::fill(std::execution::par_unseq, m_forces.begin(), m_forces.end(), glm::vec2(0.0f));
}

void CDerivativeSolver::Traverse(std::vector<size_t>& collisionList, const CBoundingBox & box)
{
	constexpr size_t kMaxStackSize = 64;
	const SAbstractNode* stack[kMaxStackSize];
	size_t top = 0;
	stack[top] = m_treeRoot;

	while (top < kMaxStackSize) //top == -1 also covered
	{
		const auto* cur = stack[top];
		--top;
		if(!cur->box.Overlaps(box))
			continue;

		if (cur->IsLeaf())
		{
			auto leaf = static_cast<const SLeafNode*>(cur);
			auto objectIdx = std::get<1>(*leaf->object);
			collisionList.push_back(objectIdx);
		}
		else
		{
			auto internalNode = static_cast<const SInternalNode*>(cur);
			stack[++top] = internalNode->left;
			if(top < kMaxStackSize)
				stack[++top] = internalNode->right;
		}
	}
}

void CDerivativeSolver::ParticleToParticle()
{
	const auto& state = m_simulation.GetState();
	const auto particles = state.particles;

	const auto* pos = m_odeState->data();
	const auto* vel = pos + particles;

	const auto rad = state.particleRad;
	const auto diameter = rad * 2.0f;

	BuildTree(pos, particles);

	m_potentialCollisionsList.resize(particles);

	std::for_each(std::execution::par_unseq, m_potentialCollisionsList.begin(), m_potentialCollisionsList.end(), [&](auto& collisionList)
	{
		collisionList.clear();
		size_t i = &collisionList - m_potentialCollisionsList.data();

		const auto &p1 = pos[i];
		const auto &v1 = vel[i];
		CBoundingBox box;
		box.AddPoint(glm::vec2(p1.x - rad, p1.y - rad));
		box.AddPoint(glm::vec2(p1.x + rad, p1.y + rad));

		Traverse(collisionList, box);

		for (size_t otherObjectIdx : collisionList)
		{
			if (otherObjectIdx == i)
				continue;

			const auto& p2 = pos[otherObjectIdx];
			const auto& v2 = vel[otherObjectIdx];

			auto force = ComputeForce(p1, v1, p2, v2, diameter);
			m_forces[i] -= force;
		}
	});
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

	m_potentialCollisionsList.resize(particles);

	std::for_each(std::execution::par_unseq, m_potentialCollisionsList.begin(), m_potentialCollisionsList.end(), [&](auto& collisionList)
	{
		collisionList.clear();
		size_t i = &collisionList - m_potentialCollisionsList.data();

		const auto &p = pos[i];
		const auto &v = vel[i];
		CBoundingBox box;
		box.AddPoint(glm::vec2(p.x - rad, p.y - rad));
		box.AddPoint(glm::vec2(p.x + rad, p.y + rad));

		Traverse(collisionList, box);

		for (size_t wingIdx : collisionList)
		{
			const auto& wp = wingParticles[wingIdx];
			m_forces[i] -= ComputeForce(p, v, wp, glm::vec2(0.0f), diameter);
		}
	});
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

ptrdiff_t CDerivativeSolver::Delta(size_t i, size_t j) const
{
	auto n = m_sortedMortonCodes.size();
	if (j > n - 1)
		return -1;

	auto firstCode = std::get<0>(m_sortedMortonCodes[i]);
	auto lastCode = std::get<0>(m_sortedMortonCodes[j]);
	auto bias = ptrdiff_t(0);

	if (firstCode == lastCode)
	{
		firstCode = i;
		lastCode = j;
		bias = 64;
	}

	auto commonPrefix = clz64(firstCode ^ lastCode);
	return bias + ptrdiff_t(commonPrefix);
}

size_t CDerivativeSolver::FindSplit(size_t i, size_t j) const
{
	auto commonPrefix = Delta(i, j);
	auto delta = ptrdiff_t(j) - ptrdiff_t(i);
	auto d = glm::sign(delta);

	auto shift = size_t(0);
	auto step = size_t(d * delta); //always positive
	do
	{
		step = (step + 1) >> 1; // exponential decrease
		if (Delta(i, i + d * (shift + step)) > commonPrefix)
			shift += step;

	} while (step > 1);

	return i + shift * d + std::min<ptrdiff_t>(d, 0);
}

size_t CDerivativeSolver::FindUpperBound(size_t i, ptrdiff_t d, ptrdiff_t dMin) const
{
	auto lMax = size_t(2);
	while (Delta(i, i + lMax * d) > dMin)
		lMax *= 2;

	auto shift = size_t(0);
	auto step = lMax;
	do
	{
		step = (step + 1) >> 1;
		if (Delta(i, i + (shift + step) * d) > dMin)
			shift += step;
	} while (step > 1);

	return i + shift * d;
}

void CDerivativeSolver::ProcessInternalNode(size_t i)
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
	self->visited = false;
}

void CDerivativeSolver::BuildTree(const glm::vec2* pos, size_t particlesCount)
{
	CBoundingBox particlesBox;
	particlesBox.AddPoints(pos, particlesCount);

	m_sortedMortonCodes.resize(particlesCount);
	std::transform(std::execution::par_unseq, pos, pos + particlesCount, m_sortedMortonCodes.begin(), [&](const auto& p)
	{
		auto i = &p - pos;
		auto normalized = (p - particlesBox.min()) / particlesBox.size();

		constexpr float kMaxSafeInt = 16777215;// 2 ^ 24 - 1

		uint32_t x = uint32_t(glm::round(normalized.x * kMaxSafeInt));
		uint32_t y = uint32_t(glm::round(normalized.y * kMaxSafeInt));
		uint64_t morton = Morton2D(x, y);

		return std::make_tuple(morton, i);
	});

	std::sort(std::execution::par_unseq, m_sortedMortonCodes.begin(), m_sortedMortonCodes.end(), [](const auto& t1, const auto& t2)
	{
		return std::get<0>(t1) < std::get<0>(t2);
	});

	const auto internalCount = particlesCount - 1;

	m_internalNodesPool.clear();
	m_leafNodesPool.clear();

	m_internalNodesPool.resize(internalCount);
	m_leafNodesPool.resize(particlesCount);

	std::for_each(std::execution::par_unseq, m_leafNodesPool.begin(), m_leafNodesPool.end(), [&](auto& leaf)
	{
		auto idx = &leaf - m_leafNodesPool.data();
		leaf.object = &m_sortedMortonCodes[idx];
	});

	std::for_each(std::execution::par_unseq, m_internalNodesPool.cbegin(), m_internalNodesPool.cend(), [&](const auto& node)
	{
		auto i = &node - m_internalNodesPool.data();
		ProcessInternalNode(i);
	});

	if (particlesCount > 1)
		m_treeRoot = &m_internalNodesPool[0];
	else
		m_treeRoot = &m_leafNodesPool[0];

	ConstructBoundingBoxes(pos, m_simulation.GetState().particleRad);
}

void CDerivativeSolver::ConstructBoundingBoxes(const glm::vec2* pos, float rad)
{
	//int64_t leafs = m_leafNodesPool.size();

	//for (int64_t i = 0; i < leafs; ++i)
	//{
	//	auto p = pos[std::get<1>(*m_leafNodesPool[i].object)];
	//	auto& box = m_leafNodesPool[i].box;
	//	box.AddPoint(glm::vec2(p.x + rad, p.y + rad));
	//	box.AddPoint(glm::vec2(p.x - rad, p.y - rad));
	//}

	//std::function<void(SAbstractNode* subTree)> recursive;

	//recursive = [&](SAbstractNode* subTree)
	//{
	//	if (subTree->IsLeaf())
	//		return;

	//	SInternalNode* self = static_cast<SInternalNode*>(subTree);
	//	if (self->left)
	//	{
	//		recursive(self->left);
	//		self->box.AddBox(self->left->box);
	//	}
	//	if (self->right)
	//	{
	//		recursive(self->right);
	//		self->box.AddBox(self->right->box);
	//	}
	//};

	//recursive(m_treeRoot);

	std::for_each(std::execution::par_unseq, m_leafNodesPool.begin(), m_leafNodesPool.end(), [&](auto& leaf)
	{
		auto p = pos[std::get<1>(*leaf.object)];
		leaf.box.AddPoint(glm::vec2(p.x + rad, p.y + rad));
		leaf.box.AddPoint(glm::vec2(p.x - rad, p.y - rad));

		SAbstractNode* cur = &leaf;
		SInternalNode* parent = static_cast<SInternalNode*>(cur->parent);
		while (parent)
		{
			auto visited = parent->visited.exchange(true);
			if (!visited)
				return;

			if (parent->left)
				parent->box.AddBox(parent->left->box);
			if (parent->right)
				parent->box.AddBox(parent->right->box);

			cur = parent;
			parent = static_cast<SInternalNode*>(cur->parent);
		}
	});
}

bool CDerivativeSolver::SLeafNode::IsLeaf() const
{
	return true;
}

bool CDerivativeSolver::SInternalNode::IsLeaf() const
{
	return false;
}
