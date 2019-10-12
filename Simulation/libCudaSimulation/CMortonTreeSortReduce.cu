#include <cub/device/device_reduce.cuh>
#include <cub/device/device_radix_sort.cuh>

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