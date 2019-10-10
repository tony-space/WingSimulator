#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

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

static __global__ void GenerateMortonCodesKernel(const size_t objects, const float4* __restrict__ boxes, const float4* __restrict__ pSceneBox, uint32_t* __restrict__ codes, size_t* __restrict__ keys)
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
	keys[threadId] = size_t(threadId);
}

void CMortonTree::Build(const SBoundingBoxesSOA& leafs)
{
	EvaluateSceneBox(leafs);
	GenerateMortonCodes(leafs.boundingBoxes);
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
	CudaSafeCall(cub::DeviceReduce::Reduce(nullptr, storageBytes, boxesPtr, m_sceneBox.sceneBox.get(), int(leafs.boundingBoxes), BoxExpander(), make_float4(INFINITY, INFINITY, -INFINITY, -INFINITY)));
	m_sceneBox.m_cubReductionTempStorage.resize(storageBytes);

	CudaSafeCall(cub::DeviceReduce::Reduce(m_sceneBox.m_cubReductionTempStorage.data().get(), storageBytes, boxesPtr, m_sceneBox.sceneBox.get(), int(leafs.boundingBoxes), BoxExpander(), make_float4(INFINITY, INFINITY, -INFINITY, -INFINITY)));
}

void CMortonTree::GenerateMortonCodes(const size_t objects)
{
	m_mortonCodes.unsortedCodes.resize(objects);
	m_mortonCodes.unsortedKeys.resize(objects);
	m_mortonCodes.sortedCodes.resize(objects);
	m_mortonCodes.sortedKeys.resize(objects);

	dim3 blockDim(kBlockSize);
	dim3 gridDim(GridSize(objects, kBlockSize));
	GenerateMortonCodesKernel <<<gridDim, blockDim >>> (objects, m_sceneBox.transformedBoxes.data().get(), m_sceneBox.sceneBox.get(), m_mortonCodes.unsortedCodes.data().get(), m_mortonCodes.unsortedKeys.data().get());
	CudaCheckError();
}