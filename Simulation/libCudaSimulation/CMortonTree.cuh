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
				void Build(const SBoundingBoxSOA& leafs);
			private:
				struct
				{
					thrust::device_vector<float4> transformedBoxes;
					thrust::device_ptr<float4> sceneBox = thrust::device_malloc<float4>(1);
					thrust::device_vector<uint8_t> m_cubReductionTempStorage;
				} m_sceneBox;

				void EvaluateSceneBox(const SBoundingBoxSOA& leafs);
			};
		}
	}
}