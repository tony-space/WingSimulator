#pragma once

#include <cstdint>
#include <thrust/device_vector.h>
#include <thrust/device_malloc.h>

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
				enum class NodeType
				{
					Leaf,
					Internal
				};
				struct STreeNode
				{
					NodeType type;
					int atomicVisited;

					float4 box;
					STreeNode* parent;
					STreeNode* left;
					STreeNode* right;
				};

				void Build(const SBoundingBoxesSOA& leafs);
			private:
				struct
				{
					thrust::device_vector<float4> transformedBoxes;
					thrust::device_ptr<float4> sceneBox = thrust::device_malloc<float4>(1);
					thrust::device_vector<uint8_t> m_cubReductionTempStorage;
				} m_sceneBox;

				struct
				{
					thrust::device_vector<uint32_t> unsortedCodes;
					thrust::device_vector<size_t> unsortedIndices;

					thrust::device_vector<uint32_t> sortedCodes;
					thrust::device_vector<size_t> sortedIndices;

					thrust::device_vector<uint8_t> m_cubSortTempStorage;
				} m_mortonCodes;

				struct
				{
					thrust::device_vector<STreeNode> leafNodesPool;
					thrust::device_vector<STreeNode> internalNodesPool;
					STreeNode* root = nullptr;
				} m_tree;

				void EvaluateSceneBox(const SBoundingBoxesSOA& leafs);
				void GenerateMortonCodes(const size_t objects);
				void SortMortonCodes();
				void InitTree(const SBoundingBoxesSOA& leafs);
				void BuildTree();
			};
		}
	}
}