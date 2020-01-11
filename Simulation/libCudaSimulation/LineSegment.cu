#include <vector>
#include <algorithm>

#include "LineSegment.cuh"

using namespace wing2d::simulation::cuda;

CLineSegmentsStorage::CLineSegmentsStorage(size_t count) : 
	m_first(count),
	m_second(count),
	m_ray(count),
	m_normal(count),
	m_length(count)
{
}

CLineSegmentsStorage::CLineSegmentsStorage(const Segments_t& segments) : CLineSegmentsStorage(segments.size())
{
	size_t count = segments.size();

	thrust::host_vector<float2> h_first(count);
	thrust::host_vector<float2> h_second(count);
	thrust::host_vector<float2> h_ray(count);
	thrust::host_vector<float2> h_normal(count);
	thrust::host_vector<float> h_length(count);

	for (size_t i = 0; i < count; ++i)
	{
		auto f = std::get<0>(segments[i]);
		auto s = std::get<1>(segments[i]);
		auto delta = s - f;
		auto l = length(delta);
		auto r = delta / l;
		auto n = make_float2(-r.y, r.x);

		h_first[i] = f;
		h_second[i] = s;
		h_ray[i] = r;
		h_normal[i] = n;
		h_length[i] = l;
	}

	m_first = h_first;
	m_second = h_second;
	m_ray = h_ray;
	m_normal = h_normal;
	m_length = h_length;
}

SLineSegmentsSOA CLineSegmentsStorage::get()
{
	return
	{
		m_first.size(),

		m_first.data().get(),
		m_second.data().get(),
		m_ray.data().get(),
		m_normal.data().get(),
		m_length.data().get()
	};
}