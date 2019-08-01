#include "pch.hpp"
#include "intrin.h"
#include "CMortonTree.hpp"

using namespace wing2d::simulation::cpu;

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

#ifdef _WIN64
#define clz64 __lzcnt64
#elif _WIN32
static inline size_t clz64(uint64_t value)
{
	uint32_t lowDword = static_cast<uint32_t>(value);
	uint32_t highDword = static_cast<uint32_t>(value >> 32);

	unsigned long pos;
	if (_BitScanReverse(&pos, highDword))
	{
		return 31 - pos;
	}
	else if (_BitScanReverse(&pos, lowDword))
	{
		return 63 - pos;
	}

	return 64;
}
#else
//https://en.wikipedia.org/wiki/Find_first_set#CLZ
//count leading zeros
static inline size_t clz64(uint64_t value)
{
	return 63 - size_t(glm::floor(glm::log(value) / glm::log(2)));
}
#endif

void CMortonTree::Build(const std::vector<glm::vec2>& pos, float rad)
{
	CBoundingBox particlesBox;
	particlesBox.AddPoints(pos);

	m_sortedTuples.resize(pos.size());
	m_sortedMortonCodes.resize(pos.size());
	std::transform(std::execution::par_unseq, pos.begin(), pos.end(), m_sortedTuples.begin(), [&](const auto& p)
	{
		size_t i = &p - pos.data();
		auto normalized = (p - particlesBox.min()) / particlesBox.size();

		constexpr float kMaxSafeInt = 16777215;// 2 ^ 24 - 1

		uint32_t x = uint32_t(glm::round(normalized.x * kMaxSafeInt));
		uint32_t y = uint32_t(glm::round(normalized.y * kMaxSafeInt));
		uint64_t morton = Morton2D(x, y);

		return std::make_tuple(morton, i);
	});

	std::sort(std::execution::par_unseq, m_sortedTuples.begin(), m_sortedTuples.end(), [](const auto& t1, const auto& t2)
	{
		return std::get<0>(t1) < std::get<0>(t2);
	});
	std::transform(std::execution::par_unseq, m_sortedTuples.cbegin(), m_sortedTuples.cend(), m_sortedMortonCodes.begin(), [](const auto& t)
	{
		return std::get<0>(t);
	});

	const auto internalCount = pos.size() - 1;

	m_internalNodesPool.clear();
	m_leafNodesPool.clear();

	m_internalNodesPool.resize(internalCount);
	m_leafNodesPool.resize(pos.size());

	std::for_each(std::execution::par_unseq, m_leafNodesPool.begin(), m_leafNodesPool.end(), [&](auto& leaf)
	{
		size_t idx = &leaf - m_leafNodesPool.data();
		leaf.id = idx;
	});

	std::for_each(std::execution::par_unseq, m_internalNodesPool.cbegin(), m_internalNodesPool.cend(), [&](const auto& node)
	{
		size_t i = &node - m_internalNodesPool.data();
		ProcessInternalNode(i);
	});

	if (pos.size() > 1)
		m_treeRoot = &m_internalNodesPool[0];
	else
		m_treeRoot = &m_leafNodesPool[0];

	ConstructBoundingBoxes(pos, rad);
}

ptrdiff_t CMortonTree::Delta(size_t i, size_t j) const
{
	auto n = m_sortedMortonCodes.size();
	if (j > n - 1)
		return -1;

	auto firstCode = m_sortedMortonCodes[i];
	auto lastCode = m_sortedMortonCodes[j];
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

size_t CMortonTree::FindSplit(size_t i, size_t j) const
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

size_t CMortonTree::FindUpperBound(size_t i, ptrdiff_t d, ptrdiff_t dMin) const
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

void CMortonTree::ProcessInternalNode(size_t i)
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

void CMortonTree::ConstructBoundingBoxes(const std::vector<glm::vec2>& pos, float rad)
{
	std::for_each(std::execution::par_unseq, m_leafNodesPool.begin(), m_leafNodesPool.end(), [&](auto& leaf)
	{
		auto p = pos[std::get<1>(m_sortedTuples[leaf.id])];
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

void CMortonTree::Traverse(std::vector<size_t>& collisionList, const CBoundingBox & box) const
{
	constexpr size_t kMaxStackSize = 64;
	const SAbstractNode* stack[kMaxStackSize];
	size_t top = 0;
	stack[top] = m_treeRoot;

	while (top < kMaxStackSize) //top == -1 also covered
	{
		const auto* cur = stack[top];
		--top;
		if (!cur->box.Overlaps(box))
			continue;

		if (cur->type == CMortonTree::SAbstractNode::NodeType::Leaf)
		{
			auto leaf = static_cast<const SLeafNode*>(cur);
			auto objectIdx = std::get<1>(m_sortedTuples[leaf->id]);
			collisionList.emplace_back(objectIdx);
		}
		else
		{
			auto internalNode = static_cast<const SInternalNode*>(cur);
			stack[++top] = internalNode->left;
			if (top < kMaxStackSize)
				stack[++top] = internalNode->right;
		}
	}
}