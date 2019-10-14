#pragma once

#include <thrust/device_vector.h>

#include "CudaInterfaces.cuh"

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			struct SLineSegmentsSOA
			{
				const size_t  lineSegments;
				const float2* __restrict__ first;
				const float2* __restrict__ second;
				const float2* __restrict__ ray;
				const float2* __restrict__ normal;
				const float* __restrict__ length;

				float __device__ __forceinline__ DistanceToLine(const size_t lineIdx, const float2& pos)
				{
					const auto dirToCenter = pos - first[lineIdx];
					const auto centerProj = dot(dirToCenter, ray[lineIdx]);

					return copysignf(
						sqrt(dot(dirToCenter, dirToCenter) - centerProj * centerProj),
						dot(dirToCenter, normal[lineIdx])
					);
				}

				float2 __device__ __forceinline__ ClosestPoint(const size_t lineIdx, const float2& pos) const
				{
					const auto f = first[lineIdx];
					const auto r = ray[lineIdx];
					const auto toPos = pos - f;
					const auto projection = dot(r, toPos);

					if (projection < 0.0f)
					{
						return f;
					}
					else if (projection >= 0.0f && projection <= length[lineIdx])
					{
						return f + r * projection;
					}
					else
					{
						return second[lineIdx];
					}
				}
			};

			class CLineSegmentsStorage
			{
			public:
				CLineSegmentsStorage(const Segments_t& segments);

				SLineSegmentsSOA get() const;
			private:
				thrust::device_vector<float2> m_first;
				thrust::device_vector<float2> m_second;
				thrust::device_vector<float2> m_ray;
				thrust::device_vector<float2> m_normal;
				thrust::device_vector<float> m_length;
			};
		}
	}
}