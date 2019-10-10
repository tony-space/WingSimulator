#include "BoundingBox.cuh"

using namespace wing2d::simulation::cuda;

CBoundingBoxesStorage::CBoundingBoxesStorage(size_t count)
{
	m_min.resize(count);
	m_max.resize(count);
}

SBoundingBoxSOA CBoundingBoxesStorage::get()
{
	return
	{
		m_min.size(),

		m_min.data().get(),
		m_max.data().get()
	};
}