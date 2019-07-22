#pragma once

#include <cstdint>
#include <vector>
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
				struct AbstractNode
				{
					CBoundingBox box;

					virtual bool IsLeaf() const = 0;
					virtual ~AbstractNode() {}
				};
				struct SLeafNode : AbstractNode
				{
					const MortonCode_t& object;

					SLeafNode(const CDerivativeSolver* solver, const MortonCode_t& obj);
					SLeafNode(const std::vector<glm::vec2>& wing, float rad, const MortonCode_t& obj);
					SLeafNode(const SLeafNode&) = default;
					SLeafNode(SLeafNode&&) = default;

					virtual bool IsLeaf() const override;
				};
				struct SInternalNode : AbstractNode
				{
					const AbstractNode* left = nullptr;
					const AbstractNode* right = nullptr;

					SInternalNode(const AbstractNode* l, const AbstractNode* r);
					SInternalNode(const SInternalNode&) = default;
					SInternalNode(SInternalNode&&) = default;

					virtual bool IsLeaf() const override;
				};

				const CSimulationCpu& m_simulation;
				const OdeState_t* m_odeState = nullptr;

				std::vector<glm::vec2> m_forces;
				std::vector<MortonCode_t> m_sortedMortonCodes;
				std::vector<SLeafNode> m_leafNodesPool;
				std::vector<SInternalNode> m_internalNodesPool;
				const AbstractNode* m_treeRoot = nullptr;
				std::vector<size_t> m_potentialCollisionsList;

				size_t FindSplit(size_t first, size_t last) const;
				const AbstractNode* GenerateHierarchyParticles(size_t first, size_t last);
				const AbstractNode* GenerateHierarchyWing(size_t first, size_t last);
				void TraverseRecursive(const AbstractNode* subtreeRoot, const CBoundingBox& queryBox, const SLeafNode* self = nullptr);

				void BuildTreeForParticleToParticle();
				void BuildTreeForParticleToWing();
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