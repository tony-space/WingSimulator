#pragma once

#include <thrust/device_vector.h>
#include <helper_math.h>

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			struct SBoundingBox
			{
				float2 min;
				float2 max;

				__device__ __host__ SBoundingBox() : min(make_float2(INFINITY, INFINITY)), max(make_float2(-INFINITY, -INFINITY))
				{

				}
				__device__ __host__ SBoundingBox(float2 _min, float2 _max) : min(_min), max(_max)
				{

				}

				__device__ const float2 Center() const
				{
					return (max + min) * 0.5f;
				}

				__device__ const float2 Size() const
				{
					return max - min;
				}
				__device__ bool Overlaps(const SBoundingBox& other) const
				{
					const auto& center1 = Center();
					const auto& center2 = other.Center();

					auto delta = fabs((center1 - center2) * 2.0f);
					auto totalSize = Size() + other.Size();

					return delta.x < totalSize.x && delta.y < totalSize.y;
				}

				static __device__ SBoundingBox ExpandBox(const SBoundingBox& a, const SBoundingBox& b)
				{
					return
					{
						fminf(a.min, b.min),
						fmaxf(a.max, b.max)
					};
				}
			};

			struct SBoundingBoxesAoS
			{
				const size_t  count;
				SBoundingBox* __restrict__ boxes;
				
				static SBoundingBoxesAoS Create(thrust::device_vector<SBoundingBox>& boxes);
				static const SBoundingBoxesAoS Create(const thrust::device_vector<SBoundingBox>& boxes);
			};
		}
	}
}