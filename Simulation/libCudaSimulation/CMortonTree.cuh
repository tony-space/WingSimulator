#pragma once

#include <cstdint>
#include <thrust/device_vector.h>
#include <thrust/device_malloc.h>

#include "CudaInterfaces.cuh"
#include "BoundingBox.cuh"

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			class CMortonTree
			{
			public:
				struct STreeNodeSoA
				{
					const TIndex leafs;

					int* __restrict__ atomicVisits;
					TIndex* __restrict__ parents;
					TIndex* __restrict__ lefts;
					TIndex* __restrict__ rights;
					SBoundingBox* __restrict__ boxes;
					const uint32_t* __restrict__ sortedMortonCodes;

					static __device__ ptrdiff_t Sign(ptrdiff_t x);
					__device__ ptrdiff_t Delta(size_t i, size_t j) const;
					__device__ size_t FindSplit(size_t i, size_t j) const;
					__device__ size_t FindUpperBound(size_t i, ptrdiff_t d, ptrdiff_t dMin) const;
					__device__ void ProcessInternalNode(size_t i);
					__device__ void ConstructBoundingBoxes(size_t leafId);
					__device__ void Traverse(const SBoundingBox& box, TIndex* sharedMem, size_t maxCollisionsPerElement, size_t reflexiveIdx) const;
				};

				struct SDeviceCollisions
				{
					const size_t elements;
					const size_t maxCollisionsPerElement;
					TIndex* __restrict__ internalIndices;
				};

				void Build(const thrust::device_vector<SBoundingBox>& leafs);
				const SDeviceCollisions Traverse(const thrust::device_vector<SBoundingBox>& objects, size_t maxCollisionsPerElement = 32);
				const thrust::device_vector<TIndex>& GetSortedIndices() const;
				const thrust::device_vector<SBoundingBox>& GetSortedBoxes() const;
			private:
				struct
				{
					thrust::device_ptr<SBoundingBox> sceneBox = thrust::device_malloc<SBoundingBox>(1);
					thrust::device_vector<uint8_t> cubReductionTempStorage;
				} m_sceneBox;

				struct
				{
					thrust::device_vector<uint32_t> unsortedCodes;
					thrust::device_vector<TIndex> unsortedIndices;

					thrust::device_vector<uint32_t> sortedCodes;
					thrust::device_vector<TIndex> sortedIndices;

					thrust::device_vector<SBoundingBox> sortedBoxes;

					thrust::device_vector<uint8_t> cubSortTempStorage;
				} m_mortonCodes;

				struct STreeNodesStorage
				{
					TIndex leafs;
					thrust::device_vector<int> atomicVisited;
					thrust::device_vector<TIndex> parent;
					thrust::device_vector<TIndex> left;
					thrust::device_vector<TIndex> right;
					thrust::device_vector<SBoundingBox> boxes;

					STreeNodeSoA get(const thrust::device_vector<uint32_t>& sortedCodes)
					{
						return
						{
							leafs,
							atomicVisited.data().get(),
							parent.data().get(),
							left.data().get(),
							right.data().get(),
							boxes.data().get(),
							sortedCodes.data().get()
						};
					}
				} m_tree;

				thrust::device_vector<TIndex> m_collisionIndices;

				void EvaluateSceneBox(const thrust::device_vector<SBoundingBox>& objects);
				void GenerateMortonCodes(const thrust::device_vector<SBoundingBox>& objects);
				void SortMortonCodes(const thrust::device_vector<SBoundingBox>& objects);
				void InitTree();
				void BuildTree();
			};
		}
	}
}