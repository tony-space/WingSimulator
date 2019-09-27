#pragma once

#include <vector>
#include <tuple>

#include <helper_math.h>

#include "CudaInterfaces.cuh"

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{

			class CDerivativeSolver : public IDerivativeSolver
			{
			public:
				typedef std::tuple<float2, float2> lineSegment_t;
				typedef std::vector<lineSegment_t> segments_t;

				CDerivativeSolver(size_t particles, float radius, const segments_t& airfoil, const segments_t& walls);

				virtual void Derive(const OdeState_t& curState, OdeState_t& outDerivative) override;

			private:
			};
		}
	}
}