#pragma once

#include <thrust/system/cuda/experimental/pinned_allocator.h>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <helper_math.h>

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			typedef thrust::host_vector<float, thrust::system::cuda::experimental::pinned_allocator<float>> PinnedHostVector1D_t;
			typedef thrust::host_vector<float2, thrust::system::cuda::experimental::pinned_allocator<float2>> PinnedHostVector2D_t;
			typedef thrust::host_vector<float4, thrust::system::cuda::experimental::pinned_allocator<float4>> PinnedHostVector4D_t;

			typedef thrust::device_vector<float2> OdeState_t;

			typedef std::tuple<float2, float2> LineSegment_t;
			typedef std::vector<LineSegment_t> Segments_t;

			struct IDerivativeSolver
			{
				virtual void Derive(const OdeState_t& curState, OdeState_t& outDerivative) = 0;
				virtual ~IDerivativeSolver() = default;
			};

			struct IOdeSolver
			{
				virtual void NextState(const thrust::device_ptr<float>& dt, const OdeState_t& curState, OdeState_t& outNextState) = 0;
				virtual ~IOdeSolver() = default;
			};
		}
	}
}