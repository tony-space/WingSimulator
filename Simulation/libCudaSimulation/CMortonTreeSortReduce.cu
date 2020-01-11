#include <cub/device/device_reduce.cuh>
#include <cub/device/device_radix_sort.cuh>

#include "CudaInterfaces.cuh"
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

static __global__ void OrderBoundingBoxesKernel(const SBoundingBox* __restrict__ unordered, TIndex* sortedIndices, size_t count, SBoundingBox* __restrict__ ordered)
{
	const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadId >= count)
		return;

	ordered[threadId] = unordered[sortedIndices[threadId]];
}

void CMortonTree::EvaluateSceneBox(const thrust::device_vector<SBoundingBox>& objects)
{
	size_t storageBytes = 0;
	CudaSafeCall(cub::DeviceReduce::Reduce(nullptr, storageBytes,
				objects.data().get(), m_sceneBox.sceneBox.get(), int(objects.size()), BoxExpander(), SBoundingBox()));

	m_sceneBox.cubReductionTempStorage.resize(storageBytes);

	CudaSafeCall(cub::DeviceReduce::Reduce(m_sceneBox.cubReductionTempStorage.data().get(), storageBytes,
				objects.data().get(), m_sceneBox.sceneBox.get(), int(objects.size()), BoxExpander(), SBoundingBox()));
}

void CMortonTree::SortMortonCodes(const thrust::device_vector<SBoundingBox>& objects)
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


	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(objects.size(), kBlockSize));
	OrderBoundingBoxesKernel <<<gridDim, blockDim >>> (
		objects.data().get(),
		m_mortonCodes.sortedIndices.data().get(),
		objects.size(),
		m_mortonCodes.sortedBoxes.data().get());
	CudaCheckError();
}