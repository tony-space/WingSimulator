#pragma once

#include "CMortonTree.cuh"

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			template<typename TDeviceCollisionResponseSolver, size_t kTreeStackSize>
			__global__ void TraverseTreeKernel(const CMortonTree::STreeNodeSoA treeInfo, TDeviceCollisionResponseSolver solver)
			{
				const auto leafId = blockIdx.x * blockDim.x + threadIdx.x;
				if (leafId >= treeInfo.leafs)
					return;

				const auto internalCount = treeInfo.leafs - 1;
				const auto leafBox = treeInfo.boxes[internalCount + leafId];

				TIndex stack[kTreeStackSize];
				unsigned top = 0;
				stack[top] = 0;

				auto deviceSideSolver = solver.Create();
				deviceSideSolver.OnPreTraversal(leafId);

				while (top < kTreeStackSize) //top == -1 also covered
				{
					auto cur = stack[top--];
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
							stack[++top] = treeInfo.lefts[cur];
							if (top + 1 < kTreeStackSize)
							{
								stack[++top] = treeInfo.rights[cur];
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
				dim3 blockDim(kBlockSize);
				dim3 gridDim(GridSize(m_tree.leafs, kBlockSize));

				TraverseTreeKernel <TDeviceCollisionResponseSolver, kTreeStackSize> <<<gridDim, blockDim >>> (m_tree.get(m_mortonCodes.sortedCodes), solver);
			}
		}
	}
}