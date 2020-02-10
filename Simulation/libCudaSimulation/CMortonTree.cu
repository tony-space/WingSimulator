#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "CudaLaunchHelpers.cuh"
#include "CMortonTree.cuh"

using namespace wing2d::simulation::cuda;

__device__ ptrdiff_t CMortonTree::STreeNodeSoA::Sign(ptrdiff_t x)
{
	if (x >= 0)
		return 1;
	else
		return -1;
}

__device__ ptrdiff_t CMortonTree::STreeNodeSoA::Delta(size_t i, size_t j) const
{
	if (j >= leafs)
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

__device__ size_t CMortonTree::STreeNodeSoA::FindSplit(size_t i, size_t j) const
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

__device__ size_t CMortonTree::STreeNodeSoA::FindUpperBound(size_t i, ptrdiff_t d, ptrdiff_t dMin) const
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

__device__ void CMortonTree::STreeNodeSoA::ProcessInternalNode(size_t i)
{
	const auto d = Sign(Delta(i, i + 1) - Delta(i, i - 1));
	const auto dMin = Delta(i, i - d);

	const auto j = FindUpperBound(i, d, dMin);
	const auto splitPos = FindSplit(i, j);

	const auto internalNodes = leafs - 1;

	TIndex left;
	TIndex right;

	if (min(i, j) == splitPos)
	{
		left = internalNodes + splitPos;
	}
	else
	{
		left = splitPos;
	}

	if (max(i, j) == splitPos + 1)
	{
		right = internalNodes + splitPos + 1;
	}
	else
	{
		right = splitPos + 1;
	}

	lefts[i] = left;
	rights[i] = right;
	parents[left] = i;
	parents[right] = i;
}

__device__ void CMortonTree::STreeNodeSoA::BottomToTopInitialization(size_t leafId)
{
	auto cur = leafs - 1 + leafId;
	auto curBox = boxes[cur];

	while (cur != 0)
	{
		auto parent = parents[cur];
		auto visited = atomicExch(&atomicVisits[parent], 1);
		if (!visited)
			return;

		TIndex siblingIndex;
		SBoundingBox siblingBox;

		TIndex rightmostIndex;
		TIndex rightmostChild;

		auto leftParentChild = lefts[parent];
		if (leftParentChild == cur)
		{
			auto rightParentChild = rights[parent];
			siblingIndex = rightParentChild;
			rightmostIndex = rightParentChild;
		}
		else
		{
			siblingIndex = leftParentChild;
			rightmostIndex = cur;
		}

		siblingBox = boxes[siblingIndex];
		rightmostChild = rightmosts[rightmostIndex];

		SBoundingBox parentBox = SBoundingBox::ExpandBox(curBox, siblingBox);
		boxes[parent] = parentBox;
		rightmosts[parent] = rightmostChild;

		cur = parent;
		curBox = parentBox;
	}
}

__device__ void CMortonTree::STreeNodeSoA::Traverse(const SBoundingBox& box, TIndex * sharedMem, size_t maxCollisionsPerElement, size_t reflexiveIdx) const
{
	const auto internalNodes = leafs - 1;

	constexpr size_t kMaxStackSize = 32;
	TIndex stack[kMaxStackSize];
	unsigned top = 0;
	stack[top] = 0;

	for (size_t i = 0; i < maxCollisionsPerElement; ++i)
		sharedMem[blockDim.x * i + threadIdx.x] = TIndex(-1);

	auto collisionIdx = 0;

	while (top < kMaxStackSize) //top == -1 also covered
	{
		auto cur = stack[top--];

		if (boxes[cur].Overlaps(box))
		{
			if (cur >= internalNodes) //if leaf
			{
				const auto leafIdx = cur - internalNodes;

				if (leafIdx == reflexiveIdx)
					continue;

				sharedMem[blockDim.x * collisionIdx + threadIdx.x] = leafIdx;
				collisionIdx++;

				if (collisionIdx >= maxCollisionsPerElement)
				{
					printf("collisions list size exceeded\n");
					return;
				}
			}
			else
			{
				stack[++top] = lefts[cur];
				if (top + 1 < kMaxStackSize)
				{
					stack[++top] = rights[cur];
				}
				else
				{
					printf("stack size exceeded\n");
				}
			}
		}
	}
}


static __device__ uint32_t ExpandBy1(uint16_t val)
{
	uint32_t result = val;

	result = ((result << 8) | result) & 0x00FF00FF;
	result = ((result << 4) | result) & 0x0F0F0F0F;
	result = ((result << 2) | result) & 0x33333333;
	result = ((result << 1) | result) & 0x55555555;

	return result;
}

static __device__ uint32_t Morton2D(uint16_t x, uint16_t y)
{
	return (ExpandBy1(x) << 1) | ExpandBy1(y);
}

static __global__ void GenerateMortonCodesKernel(const SBoundingBoxesAoS objects, const SBoundingBox* __restrict__ pSceneBox, uint32_t* __restrict__ codes, TIndex* __restrict__ indices)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= objects.count)
		return;

	auto sceneBox = *pSceneBox;
	auto sceneCenter = sceneBox.Center();
	auto sceneSize = sceneBox.Size();

	auto objectBox = objects.boxes[threadId];
	auto objectCenter = objectBox.Center();

	auto normalized = (objectCenter - sceneBox.min) / sceneSize;

	constexpr float kMaxPossibleInt = (1 << 16) - 1;
	auto x = uint16_t(lrintf(normalized.x * kMaxPossibleInt));
	auto y = uint16_t(lrintf(normalized.y * kMaxPossibleInt));
	
	codes[threadId] = Morton2D(x, y);
	indices[threadId] = TIndex(threadId);
}

static __global__ void InitTreeNodesKernel(CMortonTree::STreeNodeSoA treeInfo, const SBoundingBox* __restrict__ leafBoxes)
{
	const auto internalNodes = treeInfo.leafs - 1;
	const auto totalCount = internalNodes + treeInfo.leafs;
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;

	if (threadId < internalNodes)
	{
		treeInfo.atomicVisits[threadId] = 0;
	}
	else if (threadId < totalCount)
	{
		const auto leafId = threadId - internalNodes;
		treeInfo.boxes[threadId] = leafBoxes[leafId];
		treeInfo.rightmosts[threadId] = leafId;
	}
}

static __global__ void ProcessInternalNodesKernel(CMortonTree::STreeNodeSoA treeInfo)
{
	const auto internalNodes = treeInfo.leafs - 1;
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= internalNodes)
		return;

	treeInfo.ProcessInternalNode(threadId);
}

static __global__ void BottomToTopInitializationKernel(CMortonTree::STreeNodeSoA treeInfo)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= treeInfo.leafs)
		return;

	treeInfo.BottomToTopInitialization(threadId);
}

static __global__ void TraverseTreeKernel(const CMortonTree::STreeNodeSoA treeInfo, const SBoundingBoxesAoS externalObjects, bool reflexive, CMortonTree::SDeviceCollisions outResult)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= externalObjects.count)
		return;

	extern __shared__ TIndex collisions[];

	auto box = externalObjects.boxes[threadId];
	auto reflexiveIdx = size_t(-1);
	if (reflexive)
		reflexiveIdx = threadId;

	treeInfo.Traverse(box, collisions, outResult.maxCollisionsPerElement, reflexiveIdx);

	for (size_t i = 0; i < outResult.maxCollisionsPerElement; ++i)
	{
		const auto leafIdx = collisions[blockDim.x * i + threadIdx.x];
		outResult.internalIndices[outResult.elements * i + threadId] = leafIdx;
		if (leafIdx == TIndex(-1))
			break;
	}
}

//
//
//
void CMortonTree::Build(const thrust::device_vector<SBoundingBox>& leafs)
{
	const auto leafsCount = leafs.size();

	m_mortonCodes.unsortedCodes.resize(leafsCount);
	m_mortonCodes.unsortedIndices.resize(leafsCount);
	m_mortonCodes.sortedCodes.resize(leafsCount);
	m_mortonCodes.sortedIndices.resize(leafsCount);
	m_mortonCodes.sortedBoxes.resize(leafsCount);
	   
	EvaluateSceneBox(leafs);
	GenerateMortonCodes(leafs);
	SortMortonCodes(leafs);
	InitTree();
	BuildTree();
}

void CMortonTree::GenerateMortonCodes(const thrust::device_vector<SBoundingBox>& objects)
{
	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(objects.size(), kBlockSize));
	GenerateMortonCodesKernel <<<gridDim, blockDim >>> (SBoundingBoxesAoS::Create(objects), m_sceneBox.sceneBox.get(), m_mortonCodes.unsortedCodes.data().get(), m_mortonCodes.unsortedIndices.data().get());
	CudaCheckError();
}

void CMortonTree::InitTree()
{
	const auto leafsCount = m_mortonCodes.sortedBoxes.size();
	const auto internalCount = leafsCount - 1;
	const auto totalNodesCount = internalCount + leafsCount;

	m_tree.leafs = TIndex(leafsCount);
	m_tree.atomicVisited.resize(internalCount);
	m_tree.parent.resize(totalNodesCount);
	m_tree.left.resize(internalCount);
	m_tree.right.resize(internalCount);
	m_tree.rightmost.resize(totalNodesCount);
	m_tree.boxes.resize(totalNodesCount);

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(totalNodesCount, kBlockSize));
	InitTreeNodesKernel <<<gridDim, blockDim >>> (m_tree.get(m_mortonCodes.sortedCodes), m_mortonCodes.sortedBoxes.data().get());
	CudaCheckError();
}

void CMortonTree::BuildTree()
{
	dim3 blockDim(kBlockSize);
	dim3 gridDim;

	auto treeInfo = m_tree.get(m_mortonCodes.sortedCodes);

	gridDim = dim3(GridSize(treeInfo.leafs - 1, kBlockSize));
	ProcessInternalNodesKernel <<<gridDim, blockDim >>> (treeInfo);

	gridDim = dim3(GridSize(treeInfo.leafs, kBlockSize));
	BottomToTopInitializationKernel <<<gridDim, blockDim >>> (treeInfo);

	CudaCheckError();
}

const CMortonTree::SDeviceCollisions CMortonTree::Traverse(const thrust::device_vector<SBoundingBox>& objects, size_t maxCollisionsPerElement)
{
	const auto externalCount = objects.size();
	dim3 blockDim(32);
	dim3 gridDim(GridSize(externalCount, blockDim.x));

	auto treeInfo = m_tree.get(m_mortonCodes.sortedCodes);

	const auto capactity = externalCount * maxCollisionsPerElement;
	m_collisionIndices.resize(capactity);

	SDeviceCollisions collisions =
	{
		externalCount,
		maxCollisionsPerElement,
		m_collisionIndices.data().get()
	};

	auto reflexiveMode = &objects == &m_mortonCodes.sortedBoxes;

	TraverseTreeKernel <<<gridDim, blockDim, sizeof(TIndex) * blockDim.x * maxCollisionsPerElement>>> (treeInfo, SBoundingBoxesAoS::Create(objects), reflexiveMode, collisions);

	CudaCheckError();

	return collisions;
}

const thrust::device_vector<TIndex>& CMortonTree::GetSortedIndices() const
{
	return m_mortonCodes.sortedIndices;
}

const thrust::device_vector<SBoundingBox>& CMortonTree::GetSortedBoxes() const
{
	return m_mortonCodes.sortedBoxes;
}