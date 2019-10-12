#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "CudaLaunchHelpers.cuh"
#include "CMortonTree.cuh"

using namespace wing2d::simulation::cuda;

struct STreeInfo
{
	const size_t internalNodesCount;
	const size_t leafNodesCount;
	CMortonTree::STreeNode* __restrict__ internalNodes;
	CMortonTree::STreeNode* __restrict__ leafNodes;
	const uint32_t* const __restrict__ sortedMortonCodes;
	const size_t* const __restrict__ sortedIndices;
	const float4* const __restrict__ objectBoxes;

	__device__ __forceinline__ ptrdiff_t Delta(size_t i, size_t j) const
	{
		if (j >= leafNodesCount)
			return -1;

		auto firstCode = sortedMortonCodes[i];
		auto lastCode = sortedMortonCodes[j];
		auto bias = ptrdiff_t(0);

		if (firstCode == lastCode)
		{
			firstCode = i;
			lastCode = j;
			bias = 32;
		}

		auto commonPrefix = __clz(firstCode ^ lastCode);
		return bias + ptrdiff_t(commonPrefix);
	}

	static __device__ __forceinline__ ptrdiff_t Sign(ptrdiff_t x)
	{
		if (x >= 0)
			return 1;
		else
			return -1;
	}

	__device__ __forceinline__ size_t FindSplit(size_t i, size_t j) const
	{
		auto commonPrefix = Delta(i, j);
		auto delta = ptrdiff_t(j) - ptrdiff_t(i);
		auto d = Sign(delta);

		auto shift = size_t(0);
		auto step = size_t(d * delta); //always positive
		do
		{
			step = (step + 1) >> 1; // exponential decrease
			if (Delta(i, i + d * (shift + step)) > commonPrefix)
				shift += step;

		} while (step > 1);

		return i + shift * d + min(d, ptrdiff_t(0));
	}

	__device__ __forceinline__ size_t FindUpperBound(size_t i, ptrdiff_t d, ptrdiff_t dMin) const
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

	__device__ __forceinline__ void ProcessInternalNode(size_t i)
	{
		auto d = Sign(Delta(i, i + 1) - Delta(i, i - 1));
		auto dMin = Delta(i, i - d);

		auto j = FindUpperBound(i, d, dMin);
		auto splitPos = FindSplit(i, j);

		CMortonTree::STreeNode* __restrict__ left;
		CMortonTree::STreeNode* __restrict__ right;

		if (min(i, j) == splitPos)
		{
			left = &leafNodes[splitPos];
		}
		else
		{
			left = &internalNodes[splitPos];
		}

		if (max(i, j) == splitPos + 1)
		{
			right = &leafNodes[splitPos + 1];
		}
		else
		{
			right = &internalNodes[splitPos + 1];
		}

		CMortonTree::STreeNode* __restrict__ self = &internalNodes[i];

		left->parent = self;
		right->parent = self;
		self->left = left;
		self->right = right;
		self->atomicVisited = 0;
	}

	static __device__ __forceinline__ float4 ExpandBox(const float4& a, const float4& b)
	{
		return make_float4(fminf(a.x, b.x), fminf(a.y, b.y), fmaxf(a.z, b.z), fmaxf(a.w, b.w));
	}

	__device__ __forceinline__ void ConstructBoundingBoxes(size_t leafId)
	{
		auto box = objectBoxes[sortedIndices[leafId]];
		auto leaf = &leafNodes[leafId];
		leaf->box = box;

		CMortonTree::STreeNode* cur = leaf;
		CMortonTree::STreeNode* parent = cur->parent;
		while (parent)
		{
			auto visited = atomicExch(&parent->atomicVisited, 1);
			if (!visited)
				return;

			auto c = *cur;
			auto p = *parent;

			p.box = ExpandBox(p.box, c.box);

			if (p.left && p.left != cur)
				p.box = ExpandBox(p.box, p.left->box);
			if (p.right && p.right != cur)
				p.box = ExpandBox(p.box, p.right->box);

			parent->box = p.box;

			cur = parent;
			parent = c.parent;
		}
	}
};

static __device__ __forceinline__ uint32_t ExpandBy1(uint16_t val)
{
	uint32_t result = val;

	result = ((result << 8) | result) & 0x00FF00FF;
	result = ((result << 4) | result) & 0x0F0F0F0F;
	result = ((result << 2) | result) & 0x33333333;
	result = ((result << 1) | result) & 0x55555555;

	return result;
}

static __device__ __forceinline__ uint32_t Morton2D(uint16_t x, uint16_t y)
{
	return (ExpandBy1(x) << 1) | ExpandBy1(y);
}

static __global__ void GenerateMortonCodesKernel(const size_t objects, const float4* __restrict__ boxes, const float4* __restrict__ pSceneBox, uint32_t* __restrict__ codes, size_t* __restrict__ indices)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= objects)
		return;

	auto sceneBox = *pSceneBox;
	auto sceneMin = make_float2(sceneBox.x, sceneBox.y);
	auto sceneMax = make_float2(sceneBox.z, sceneBox.w);
	auto sceneCenter = (sceneMin + sceneMax) * 0.5f;
	auto sceneSize = sceneMax - sceneMin;

	auto objectBox = boxes[threadId];
	auto objectCenter = make_float2((objectBox.x + objectBox.z) * 0.5f, (objectBox.y + objectBox.w) * 0.5f);

	auto normalized = (objectCenter - sceneMin) / sceneSize;

	constexpr float kMaxPossibleInt = (1 << 16) - 1;
	auto x = uint16_t(lrintf(normalized.x * kMaxPossibleInt));
	auto y = uint16_t(lrintf(normalized.y * kMaxPossibleInt));
	
	codes[threadId] = Morton2D(x, y);
	indices[threadId] = size_t(threadId);
}

static __global__ void InitTreeNodesPoolKernel(size_t count, CMortonTree::NodeType type, CMortonTree::STreeNode* __restrict__ nodes, float2* const __restrict__ pMin, float2* const __restrict__ pMax)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= count)
		return;

	CMortonTree::STreeNode result;
	result.type = type;
	result.left = nullptr;
	result.right = nullptr;
	result.parent = nullptr;
	result.atomicVisited = 0;
	
	if (type == CMortonTree::NodeType::Leaf)
	{
		auto min = pMin[threadId];
		auto max = pMax[threadId];
		result.box = make_float4(min.x, min.y, max.x, max.y);
	}
	else
	{
		result.box = make_float4(INFINITY, INFINITY, -INFINITY, -INFINITY);
	}

	nodes[threadId] = result;
}

static __global__ void ProcessInternalNodesKernel(STreeInfo treeInfo)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= treeInfo.internalNodesCount)
		return;

	treeInfo.ProcessInternalNode(threadId);
}

static __global__ void ConstructBoundingBoxesKernel(STreeInfo treeInfo)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= treeInfo.leafNodesCount)
		return;

	treeInfo.ConstructBoundingBoxes(threadId);
}

//
//
//
void CMortonTree::Build(const SBoundingBoxesSOA& leafs)
{
	EvaluateSceneBox(leafs);
	GenerateMortonCodes(leafs.boundingBoxes);
	SortMortonCodes();
	InitTree(leafs);
	BuildTree();
}

void CMortonTree::GenerateMortonCodes(const size_t objects)
{
	m_mortonCodes.unsortedCodes.resize(objects);
	m_mortonCodes.unsortedIndices.resize(objects);
	m_mortonCodes.sortedCodes.resize(objects);
	m_mortonCodes.sortedIndices.resize(objects);

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(objects, kBlockSize));
	GenerateMortonCodesKernel <<<gridDim, blockDim >>> (objects, m_sceneBox.transformedBoxes.data().get(), m_sceneBox.sceneBox.get(), m_mortonCodes.unsortedCodes.data().get(), m_mortonCodes.unsortedIndices.data().get());
	CudaCheckError();
}

void CMortonTree::InitTree(const SBoundingBoxesSOA& leafs)
{
	const auto leafsCount = leafs.boundingBoxes;
	const auto internalCount = leafsCount - 1;

	m_tree.internalNodesPool.resize(internalCount);
	m_tree.leafNodesPool.resize(leafsCount);

	if (leafs.boundingBoxes > 1)
		m_tree.root = m_tree.internalNodesPool.data().get();
	else
		m_tree.root = m_tree.leafNodesPool.data().get();

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(leafsCount, kBlockSize));
	InitTreeNodesPoolKernel <<<gridDim, blockDim >>> (internalCount, NodeType::Internal, m_tree.internalNodesPool.data().get(), nullptr, nullptr);
	InitTreeNodesPoolKernel <<<gridDim, blockDim >>> (leafsCount, NodeType::Leaf, m_tree.leafNodesPool.data().get(), leafs.min, leafs.max);
	CudaCheckError();
}

void CMortonTree::BuildTree()
{
	const auto internalCount = m_tree.internalNodesPool.size();
	const auto leafsCount = m_tree.leafNodesPool.size();

	dim3 blockDim(kBlockSize);
	dim3 gridDim;

	STreeInfo info = {
		internalCount,
		leafsCount,
		m_tree.internalNodesPool.data().get(),
		m_tree.leafNodesPool.data().get(),
		m_mortonCodes.sortedCodes.data().get(),
		m_mortonCodes.sortedIndices.data().get(),
		m_sceneBox.transformedBoxes.data().get()
	};

	gridDim = dim3(GridSize(internalCount, kBlockSize));
	ProcessInternalNodesKernel <<<gridDim, blockDim >>> (info);

	gridDim = dim3(GridSize(leafsCount, kBlockSize));
	ConstructBoundingBoxesKernel <<<gridDim, blockDim >>> (info);

	CudaCheckError();
}