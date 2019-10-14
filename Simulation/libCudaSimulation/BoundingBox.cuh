#pragma once

#include <thrust/device_vector.h>
#include <helper_math.h>

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			struct SBoundingBoxesSOA
			{
				const size_t  boundingBoxes;
				float2* __restrict__ min;
				float2* __restrict__ max;
			};

			class CBoundingBoxesStorage
			{
			public:
				CBoundingBoxesStorage(size_t count);
				SBoundingBoxesSOA get();
			private:
				thrust::device_vector<float2> m_min;
				thrust::device_vector<float2> m_max;
			};
		}
	}
}