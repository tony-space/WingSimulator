#pragma once

#include <cstdint>
#include <vector>
#include <tuple>

#include "CBoundingBox.hpp"

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CMortonTree
			{
			public:
				void Build(const std::vector<CBoundingBox>& leafs);
				void Traverse(const CBoundingBox& box, std::vector<size_t>& outCollisionList) const;
			private:
				typedef std::tuple<uint64_t, size_t> MortonCode_t;
				struct SAbstractNode
				{
					enum class NodeType
					{
						Leaf,
						Internal
					};

					NodeType type;
					CBoundingBox box;
					SAbstractNode* parent = nullptr;

					SAbstractNode(NodeType _type) : type(_type)
					{

					}
				};
				struct SLeafNode : SAbstractNode
				{
					size_t id = 0;

					SLeafNode() : SAbstractNode(NodeType::Leaf) {}
				};
				struct SInternalNode : SAbstractNode
				{
					SAbstractNode* left = nullptr;
					SAbstractNode* right = nullptr;
					std::atomic_bool visited = false;


					SInternalNode() : SAbstractNode(NodeType::Internal)
					{
					};
					SInternalNode(const SInternalNode& other) : SAbstractNode(SAbstractNode::NodeType::Internal)
					{
					};
				};

				std::vector<MortonCode_t> m_sortedTuples;
				std::vector<uint64_t> m_sortedMortonCodes;

				std::vector<SLeafNode> m_leafNodesPool;
				std::vector<SInternalNode> m_internalNodesPool;
				SAbstractNode* m_treeRoot = nullptr;

				ptrdiff_t Delta(size_t i, size_t j) const;
				size_t FindSplit(size_t i, size_t j) const;
				size_t FindUpperBound(size_t i, ptrdiff_t d, ptrdiff_t dMin) const;
				void ProcessInternalNode(size_t i);
				void ConstructBoundingBoxes(const std::vector<CBoundingBox>& leafs);
			};
		}
	}
}