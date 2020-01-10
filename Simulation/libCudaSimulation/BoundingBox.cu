#include "BoundingBox.cuh"

using namespace wing2d::simulation::cuda;

CBoundingBoxesStorage::CBoundingBoxesStorage(size_t count)
{
	m_boxes.resize(count);
}

SBoundingBoxesAoS CBoundingBoxesStorage::get()
{
	return
	{
		m_boxes.size(),

		m_boxes.data().get()
	};
}