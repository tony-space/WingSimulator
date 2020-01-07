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

					SBoundingBox box;
					STreeNode* __restrict__ parent;
					STreeNode* __restrict__ left;
					STreeNode* __restrict__ right;
				};

				struct SDeviceCollisions
				{
					size_t capacity;
					size_t* __restrict__ internalIndices;
				};

				void Build(const SBoundingBoxesSOA& leafs);
				const SDeviceCollisions Traverse(const SBoundingBoxesSOA& objects);
			private:
				struct
				{
					thrust::device_vector<SBoundingBox> transformedBoxes;
					thrust::device_ptr<SBoundingBox> sceneBox = thrust::device_malloc<SBoundingBox>(1);
					thrust::device_vector<uint8_t> cubReductionTempStorage;
				} m_sceneBox;

				struct
				{
					thrust::device_vector<uint32_t> unsortedCodes;
					thrust::device_vector<size_t> unsortedIndices;

					thrust::device_vector<uint32_t> sortedCodes;
					thrust::device_vector<size_t> sortedIndices;

					thrust::device_vector<uint8_t> cubSortTempStorage;
				} m_mortonCodes;

				struct
				{
					thrust::device_vector<STreeNode> leafNodesPool;
					thrust::device_vector<STreeNode> internalNodesPool;
					STreeNode* root = nullptr;
				} m_tree;

				struct
				{
					thrust::device_vector<size_t> internalIndices;
				} m_collisions;

				void EvaluateSceneBox(const SBoundingBoxesSOA& leafs);
				void GenerateMortonCodes(const size_t objects);
				void SortMortonCodes();
				void InitTree(const SBoundingBoxesSOA& leafs);
				void BuildTree();
			};
		}
	}
}