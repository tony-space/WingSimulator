#pragma once

#include <cstdint>
#include <vector>
#include <atomic>
#include <glm/glm.hpp>

#include "CBoundingBox.hpp"

namespace wing2d
{
	namespace simulation
	{
		namespace cpu
		{
			class CSimulationCpu;

			class CDerivativeSolver
			{
			public:
				typedef std::vector<glm::vec2> OdeState_t;

				CDerivativeSolver(const CSimulationCpu& sim);
				void operator() (const OdeState_t& odeState, OdeState_t& odeDerivative);
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

				const CSimulationCpu& m_simulation;
				const OdeState_t* m_odeState = nullptr;

				std::vector<glm::vec2> m_forces;
				std::vector<MortonCode_t> m_sortedMortonCodes;
				std::vector<SLeafNode> m_leafNodesPool;
				std::vector<SInternalNode> m_internalNodesPool;
				SAbstractNode* m_treeRoot = nullptr;
				std::vector<std::vector<size_t>> m_potentialCollisionsList;

				void Traverse(std::vector<size_t>& collisionList, const CBoundingBox& box);

				ptrdiff_t Delta(size_t i, size_t j) const;
				size_t FindSplit(size_t i, size_t j) const;
				size_t FindUpperBound(size_t i, ptrdiff_t d, ptrdiff_t dMin) const;
				void ProcessInternalNode(size_t i);
				void BuildTree(const glm::vec2* pos, size_t particlesCount);

				void ConstructBoundingBoxes(const glm::vec2* pos, float rad);

				void ResetForces();
				void ParticleToParticle();
				void ParticleToWing();
				void ParticleToWall();
				void ApplyGravity();

				static glm::vec2 ComputeForce(const glm::vec2& pos1, const glm::vec2& vel1, const glm::vec2& pos2, const glm::vec2& vel2, float diameter);
			};
		}
	}
}