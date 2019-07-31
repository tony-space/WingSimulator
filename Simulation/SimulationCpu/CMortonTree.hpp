#pragma once

#include <cstdint>
#include <vector>
#include <tuple>

#include <glm/vec2.hpp>
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
				void Build(const glm::vec2* pos, size_t count, float rad);
				void Traverse(std::vector<size_t>& collisionList, const CBoundingBox & box) const;
			private:
				typedef std::tuple<uint64_t, size_t> MortonCode_t;
				struct SAbstractNode
				{
					CBoundingBox box;

					SAbstractNode* parent = nullptr;

					virtual bool IsLeaf() const = 0;
					virtual ~SAbstractNode() {}
				};
				struct SLeafNode : SAbstractNode
				{
					const MortonCode_t* object;
					virtual bool IsLeaf() const override;
				};
				struct SInternalNode : SAbstractNode
				{
					SAbstractNode* left = nullptr;
					SAbstractNode* right = nullptr;
					std::atomic_bool visited = false;


					SInternalNode() = default;
					SInternalNode(const SInternalNode& other)/* : left(other.left), right(other.right), visited((bool)other.visited)*/
					{
					};
					virtual bool IsLeaf() const override;
				};

				std::vector<MortonCode_t> m_sortedMortonCodes;
				std::vector<SLeafNode> m_leafNodesPool;
				std::vector<SInternalNode> m_internalNodesPool;
				SAbstractNode* m_treeRoot = nullptr;

				ptrdiff_t Delta(size_t i, size_t j) const;
				size_t FindSplit(size_t i, size_t j) const;
				size_t FindUpperBound(size_t i, ptrdiff_t d, ptrdiff_t dMin) const;
				void ProcessInternalNode(size_t i);
				void ConstructBoundingBoxes(const glm::vec2* pos, float rad);
			};
		}
	}
}