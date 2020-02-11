#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cooperative_groups.h>

#include "CMortonTree.cuh"

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			template<typename TDeviceCollisionResponseSolver, size_t kTreeStackSize, size_t kWarpSize = 32, size_t kCacheSize = 32>
			__global__ void TraverseMortonTreeReflexiveKernel(const CMortonTree::STreeNodeSoA treeInfo, TDeviceCollisionResponseSolver solver)
			{
				namespace cg = cooperative_groups;

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

					if (treeInfo.boxes[cur].Overlaps(leafBox))
					{
						if (cur >= internalCount) //if leaf
						{
							deviceSideSolver.OnCollisionDetected(cur - internalCount);
						}
						else
						{
							stack[++top][threadIdx.x] = treeInfo.lefts[cur];
							if (top + 1 < kTreeStackSize)
							{
								stack[++top][threadIdx.x] = treeInfo.rights[cur];
							}
							else
							{
								printf("stack size exceeded\n");
							}
						}
					}
				}

				deviceSideSolver.OnPostTraversal();
			}

			template<typename TDeviceCollisionResponseSolver, size_t kTreeStackSize>
			void CMortonTree::TraverseReflexive(const TDeviceCollisionResponseSolver& solver)
			{
				constexpr size_t kWarpSize = 32;
				dim3 blockDim(kWarpSize);
				dim3 gridDim(GridSize(m_tree.leafs, blockDim.x));

				TraverseMortonTreeReflexiveKernel <TDeviceCollisionResponseSolver, kTreeStackSize, kWarpSize> <<<gridDim, blockDim >>> (m_tree.get(m_mortonCodes.sortedCodes), solver);
				CudaCheckError();
			}
		}
	}
}