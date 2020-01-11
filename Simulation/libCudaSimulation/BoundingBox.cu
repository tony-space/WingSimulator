#include "BoundingBox.cuh"

using namespace wing2d::simulation::cuda;

SBoundingBoxesAoS SBoundingBoxesAoS::Create(thrust::device_vector<SBoundingBox>& boxes)
{
	return
	{
		boxes.size(),
		boxes.data().get()
	};
}

const SBoundingBoxesAoS SBoundingBoxesAoS::Create(const thrust::device_vector<SBoundingBox>& boxes)
{
	return
	{
		boxes.size(),
		const_cast<SBoundingBox*>(boxes.data().get())
	};
}