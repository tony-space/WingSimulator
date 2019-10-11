#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "CudaLaunchHelpers.cuh"
#include "CMortonTree.cuh"

using namespace wing2d::simulation::cuda;

struct BoxExpander
{
	__device__ __forceinline__ float4 operator()(const float4& a, const float4& b) const
	{
		return make_float4(fminf(a.x, b.x), fminf(a.y, b.y), fmaxf(a.z, b.z), fmaxf(a.w, b.w));
	}
};

static __global__ void TransformBoxesKernel(const SBoundingBoxesSOA boxes, float4* __restrict__ out)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= boxes.boundingBoxes)
		return;

	auto min = boxes.min[threadId];
	auto max = boxes.max[threadId];

	out[threadId] = make_float4(min.x, min.y, max.x, max.y);
}

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

static __global__ void ProcessInternalNodesKernel(size_t internalNodesCount, CMortonTree::STreeNode* __restrict__ internalNodes, CMortonTree::STreeNode* __restrict__ leafNodes, uint32_t* const __restrict__ sortedMortonCodes)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= internalNodesCount)
		return;
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
	ProcessInternalNodes();
	ConstructBoundingBoxes();
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

void CMortonTree::ProcessInternalNodes()
{
	const auto internalCount = m_tree.internalNodesPool.size();

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(internalCount, kBlockSize));
	ProcessInternalNodesKernel <<<gridDim, blockDim >>> (
		internalCount,
		m_tree.internalNodesPool.data().get(),
		m_tree.leafNodesPool.data().get(),
		m_mortonCodes.sortedCodes.data().get());
	CudaCheckError();
}

void CMortonTree::ConstructBoundingBoxes()
{

}

#include <cub/device/device_reduce.cuh>
#include <cub/device/device_radix_sort.cuh>

void CMortonTree::EvaluateSceneBox(const SBoundingBoxesSOA& leafs)
{
	m_sceneBox.transformedBoxes.resize(leafs.boundingBoxes);
	auto boxesPtr = m_sceneBox.transformedBoxes.data().get();

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(leafs.boundingBoxes, kBlockSize));
	TransformBoxesKernel << <gridDim, blockDim >> > (leafs, boxesPtr);
	CudaCheckError();

	size_t storageBytes = 0;
	CudaSafeCall(cub::DeviceReduce::Reduce(nullptr, storageBytes,
		boxesPtr, m_sceneBox.sceneBox.get(), int(leafs.boundingBoxes), BoxExpander(), make_float4(INFINITY, INFINITY, -INFINITY, -INFINITY)));

	m_sceneBox.m_cubReductionTempStorage.resize(storageBytes);

	CudaSafeCall(cub::DeviceReduce::Reduce(m_sceneBox.m_cubReductionTempStorage.data().get(), storageBytes,
		boxesPtr, m_sceneBox.sceneBox.get(), int(leafs.boundingBoxes), BoxExpander(), make_float4(INFINITY, INFINITY, -INFINITY, -INFINITY)));
}

void CMortonTree::SortMortonCodes()
{
	size_t storageBytes = 0;

	CudaSafeCall(cub::DeviceRadixSort::SortPairs(nullptr, storageBytes,
		m_mortonCodes.unsortedCodes.data().get(),
		m_mortonCodes.sortedCodes.data().get(),

		m_mortonCodes.unsortedIndices.data().get(),
		m_mortonCodes.sortedIndices.data().get(),
		int(m_mortonCodes.unsortedCodes.size())
	));

	m_mortonCodes.m_cubSortTempStorage.resize(storageBytes);

	CudaSafeCall(cub::DeviceRadixSort::SortPairs(m_mortonCodes.m_cubSortTempStorage.data().get(), storageBytes,
		m_mortonCodes.unsortedCodes.data().get(),
		m_mortonCodes.sortedCodes.data().get(),

		m_mortonCodes.unsortedIndices.data().get(),
		m_mortonCodes.sortedIndices.data().get(),
		int(m_mortonCodes.unsortedCodes.size())
	));
}