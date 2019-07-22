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
	BuildTree();
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

size_t CDerivativeSolver::FindSplit(size_t first, size_t last) const
{
	// Identical Morton codes => split the range in the middle.
	auto firstCode = std::get<0>(m_sortedMortonCodes[first]);
	auto lastCode = std::get<0>(m_sortedMortonCodes[last]);

	if (firstCode == lastCode)
		return (first + last) >> 1;

	// Calculate the number of highest bits that are the same
	// for all objects, using the count-leading-zeros intrinsic.
	auto commonPrefix = clz64(firstCode ^ lastCode);

	// Use binary search to find where the next bit differs.
	// Specifically, we are looking for the highest object that
	// shares more than commonPrefix bits with the first one.
	auto split = first; // initial guess
	auto step = last - first;

	do
	{
		step = (step + 1) >> 1; // exponential decrease
		auto newSplit = split + step; // proposed new position

		if (newSplit < last)
		{
			auto splitCode = std::get<0>(m_sortedMortonCodes[newSplit]);
			auto splitPrefix = clz64(firstCode ^ splitCode);
			if (splitPrefix > commonPrefix)
				split = newSplit; // accept proposal
		}
	} while (step > 1);

	return split;
}

const CDerivativeSolver::AbstractNode* CDerivativeSolver::GenerateHierarchy(size_t first, size_t last)
{
	if (first == last)
		return &m_leafNodesPool.emplace_back(this, m_sortedMortonCodes[first]);

	// Determine where to split the range.
	auto splitIdx = FindSplit(first, last);

	// Process the resulting sub-ranges recursively.
	auto childA = GenerateHierarchy(first, splitIdx);
	auto childB = GenerateHierarchy(splitIdx + 1, last);
	return &m_internalNodesPool.emplace_back(childA, childB);
}

void CDerivativeSolver::BuildTree()
{
	CBoundingBox particlesBox;

	const auto& state = m_simulation.GetState();
	const auto particles = state.particles;

	auto pos = *m_odeState | boost::adaptors::sliced(0, particles);
	particlesBox.AddPoints(pos.begin(), pos.end());
	m_sortedMortonCodes.resize(particles);
	for (size_t i = 0; i < particles; ++i)
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

	m_internalNodesPool.clear();
	m_leafNodesPool.clear();

	m_internalNodesPool.reserve(particles - 1);
	m_leafNodesPool.reserve(particles);

	m_treeRoot = GenerateHierarchy(0, particles - 1);
}

void CDerivativeSolver::ResetForces()
{
	std::fill(m_forces.begin(), m_forces.end(), glm::vec2(0.0f));
}

void CDerivativeSolver::TraverseRecursive(size_t sortedIndex, const AbstractNode* node)
{
	const auto& nodeBox = m_leafNodesPool[sortedIndex].box;
	if (!nodeBox.Overlaps(node->box))
		return;

	if (node->IsLeaf())
	{
		if (&m_leafNodesPool[sortedIndex] != node)
		{
			auto leaf = static_cast<const SLeafNode*>(node);
			auto objectIdx = std::get<1>(leaf->object);
			m_potentialCollisionsList.push_back(objectIdx);
		}
	}
	else
	{
		auto internalNode = static_cast<const SInternalNode*>(node);
		TraverseRecursive(sortedIndex, internalNode->left);
		TraverseRecursive(sortedIndex, internalNode->right);
	}
}

void CDerivativeSolver::ParticleToParticle()
{
	const auto& state = m_simulation.GetState();
	const auto particles = state.particles;

	auto pos = *m_odeState | boost::adaptors::sliced(0, particles);
	auto vel = *m_odeState | boost::adaptors::sliced(particles, particles * 2);
	const auto diameter = state.particleRad * 2.0f;

	for (size_t sortedIdx = 0; sortedIdx < particles; ++sortedIdx)
	{
		m_potentialCollisionsList.clear();

		auto index = std::get<1>(m_sortedMortonCodes[sortedIdx]);
		TraverseRecursive(sortedIdx, m_treeRoot);

		if(m_potentialCollisionsList.empty())
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
	const auto particles = state.particles;

	auto pos = *m_odeState | boost::adaptors::sliced(0, particles);
	auto vel = *m_odeState | boost::adaptors::sliced(particles, particles * 2);
	const auto diameter = state.particleRad * 2.0f;

	for (size_t i = 0; i < particles; ++i)
	{
		const auto &p = pos[i];
		const auto &v = vel[i];

		for (const auto& wp : wingParticles)
		{
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

CDerivativeSolver::SLeafNode::SLeafNode(const CDerivativeSolver* solver, const MortonCode_t & obj) : object(obj)
{
	auto rad = solver->m_simulation.GetState().particleRad;

	auto idx = std::get<1>(obj);
	auto pos = (*solver->m_odeState)[idx];

	box.AddPoint(glm::vec2(pos.x - rad, pos.y - rad));
	box.AddPoint(glm::vec2(pos.x + rad, pos.y + rad));
}

bool CDerivativeSolver::SLeafNode::IsLeaf() const
{
	return true;
}

CDerivativeSolver::SInternalNode::SInternalNode(const AbstractNode * l, const AbstractNode * r) : left(l), right(r)
{
	box.AddPoint(left->box.min());
	box.AddPoint(left->box.max());
	box.AddPoint(right->box.min());
	box.AddPoint(right->box.max());
}

bool CDerivativeSolver::SInternalNode::IsLeaf() const
{
	return false;
}
