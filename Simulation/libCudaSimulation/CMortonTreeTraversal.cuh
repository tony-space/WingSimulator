#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "CMortonTree.cuh"

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			template<typename TDeviceCollisionResponseSolver, size_t kTreeStackSize, size_t kWarpSize = 32>
			__global__ void TraverseMortonTreeReflexiveKernel(const CMortonTree::STreeNodeSoA treeInfo, TDeviceCollisionResponseSolver solver)
			{
				const auto leafId = blockIdx.x * blockDim.x + threadIdx.x;
				if (leafId >= treeInfo.leafs)
					return;

				const auto internalCount = treeInfo.leafs - 1;
				const auto leafBox = treeInfo.boxes[internalCount + leafId];

				__shared__ TIndex stack[kTreeStackSize][kWarpSize];

				unsigned top = 0;
				stack[top][threadIdx.x] = 0;

				auto deviceSideSolver = solver.Create();
				deviceSideSolver.OnPreTraversal(leafId);

				while (top < kTreeStackSize) //top == -1 also covered
				{
					auto cur = stack[top--][threadIdx.x];
					if (cur == leafId + internalCount)
						continue;

					if (treeInfo.rightmosts[cur] < leafId)
						continue;

					if (!treeInfo.boxes[cur].Overlaps(leafBox))
						continue;
					
					if (cur < internalCount)
					{
						stack[++top][threadIdx.x] = treeInfo.lefts[cur];
						if (top + 1 < kTreeStackSize)
							stack[++top][threadIdx.x] = treeInfo.rights[cur];
						else
							printf("stack size exceeded\n");

						continue;
					}
					
					deviceSideSolver.OnCollisionDetected(cur - internalCount);
				}

				deviceSideSolver.OnPostTraversal();
			}

			template<typename TDeviceCollisionResponseSolver, size_t kTreeStackSize>
			void CMortonTree::TraverseReflexive(const TDeviceCollisionResponseSolver& solver)
			{
				constexpr size_t kWarpSize = 32;
				dim3 blockDim(kWarpSize);
				dim3 gridDim(GridSize(m_tree.leafs, blockDim.x));

				TraverseMortonTreeReflexiveKernel <TDeviceCollisionResponseSolver, kTreeStackSize, kWarpSize> <<<gridDim, blockDim >>> (
					m_tree.get(m_mortonCodes.sortedCodes),
					solver);
				CudaCheckError();
			}

			template<typename TDeviceCollisionResponseSolver, size_t kTreeStackSize, size_t kWarpSize = 32>
			__global__ void TraverseMortonTreeKernel(const CMortonTree::STreeNodeSoA treeInfo, const SBoundingBoxesAoS externalObjects, TDeviceCollisionResponseSolver solver)
			{
				const auto threadId = blockIdx.x * blockDim.x + threadIdx.x;
				if (threadId >= externalObjects.count)
					return;

				const auto objectBox = externalObjects.boxes[threadId];
				const auto internalCount = treeInfo.leafs - 1;

				__shared__ TIndex stack[kTreeStackSize][kWarpSize];

				unsigned top = 0;
				stack[top][threadIdx.x] = 0;

				auto deviceSideSolver = solver.Create();
				deviceSideSolver.OnPreTraversal(threadId);

				while (top < kTreeStackSize) //top == -1 also covered
				{
					auto cur = stack[top--][threadIdx.x];

					if (!treeInfo.boxes[cur].Overlaps(objectBox))
						continue;

					if (cur < internalCount)
					{
						stack[++top][threadIdx.x] = treeInfo.lefts[cur];
						if (top + 1 < kTreeStackSize)
							stack[++top][threadIdx.x] = treeInfo.rights[cur];
						else
							printf("stack size exceeded\n");

						continue;
					}

					deviceSideSolver.OnCollisionDetected(cur - internalCount);
				}

				deviceSideSolver.OnPostTraversal();
			}

			template<typename TDeviceCollisionResponseSolver, size_t kTreeStackSize>
			void CMortonTree::Traverse(const thrust::device_vector<SBoundingBox>& objects, const TDeviceCollisionResponseSolver& solver)
			{
				constexpr size_t kWarpSize = 32;
				dim3 blockDim(kWarpSize);
				dim3 gridDim(GridSize(objects.size(), blockDim.x));

				const auto boxes = SBoundingBoxesAoS::Create(objects);

				TraverseMortonTreeKernel <TDeviceCollisionResponseSolver, kTreeStackSize, kWarpSize> <<<gridDim, blockDim >>> (
					m_tree.get(m_mortonCodes.sortedCodes),
					boxes,
					solver);
				CudaCheckError();
			}
		}
	}
}