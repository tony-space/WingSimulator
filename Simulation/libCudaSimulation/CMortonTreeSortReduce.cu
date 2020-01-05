#include <cub/device/device_reduce.cuh>
#include <cub/device/device_radix_sort.cuh>

#include "CudaLaunchHelpers.cuh"
#include "CMortonTree.cuh"

using namespace wing2d::simulation::cuda;

//You cannot just pass a pointer to a device function to CUB library, because you'll get cudaErrorInvalidPc error.
struct BoxExpander
{
	__device__ SBoundingBox operator()(const SBoundingBox& a, const SBoundingBox& b) const
	{
		return SBoundingBox::ExpandBox(a, b);
	}
};

static __global__ void TransformBoxesKernel(const SBoundingBoxesSOA boxes, SBoundingBox* __restrict__ out)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= boxes.boundingBoxes)
		return;

	auto min = boxes.min[threadId];
	auto max = boxes.max[threadId];

	out[threadId] = {min, max};
}

void CMortonTree::EvaluateSceneBox(const SBoundingBoxesSOA& leafs)
{
	m_sceneBox.transformedBoxes.resize(leafs.boundingBoxes);
	auto boxesPtr = m_sceneBox.transformedBoxes.data().get();

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(leafs.boundingBoxes, kBlockSize));
	TransformBoxesKernel <<<gridDim, blockDim >>> (leafs, boxesPtr);
	CudaCheckError();

	size_t storageBytes = 0;
	CudaSafeCall(cub::DeviceReduce::Reduce(nullptr, storageBytes,
				 boxesPtr, m_sceneBox.sceneBox.get(), int(leafs.boundingBoxes), BoxExpander(), SBoundingBox()));

	m_sceneBox.cubReductionTempStorage.resize(storageBytes);

	CudaSafeCall(cub::DeviceReduce::Reduce(m_sceneBox.cubReductionTempStorage.data().get(), storageBytes,
				 boxesPtr, m_sceneBox.sceneBox.get(), int(leafs.boundingBoxes), BoxExpander(), SBoundingBox()));
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

	m_mortonCodes.cubSortTempStorage.resize(storageBytes);

	CudaSafeCall(cub::DeviceRadixSort::SortPairs(m_mortonCodes.cubSortTempStorage.data().get(), storageBytes,
				 m_mortonCodes.unsortedCodes.data().get(),
				 m_mortonCodes.sortedCodes.data().get(),

				 m_mortonCodes.unsortedIndices.data().get(),
				 m_mortonCodes.sortedIndices.data().get(),
				 int(m_mortonCodes.unsortedCodes.size())
	));
}