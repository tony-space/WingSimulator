#include "pch.hpp"

int main()
{
	auto devices = boost::compute::system::devices();
	std::cout << "Found devices:" << std::endl;
	for (size_t i = 0; i < devices.size(); ++i)
	{
		const auto& device = devices[i];
		std::cout << i << ": \t" << device.name() << std::endl;
	}

	size_t idx;
	std::cout << "Select device: ";
	std::cin >> idx;


	//auto device = boost::compute::system::default_device();
	auto device = devices[idx];
	std::cout << "Using device: " << device.name() << std::endl;

	auto context = boost::compute::context(device);
	auto queue = boost::compute::command_queue(context, device);

	std::vector <float> data(32 * 1024 * 1024);
	for (auto& f : data)
	{
		f = float(rand());
	}

	auto vector = boost::compute::vector<float>(data.size(), context);
	boost::compute::copy(data.cbegin(), data.cend(), vector.begin(), queue);

	BOOST_COMPUTE_FUNCTION(float, foo, (float x),
	{
		return sqrt(x) + sin(x);
	});

	BOOST_COMPUTE_FUNCTION(float, bar, (float x),
	{
		return x * x;
	});

	boost::compute::transform(vector.cbegin(), vector.cend(), vector.begin(), foo, queue);
	boost::compute::transform(vector.cbegin(), vector.cend(), vector.begin(), bar, queue);
	boost::compute::sort(vector.begin(), vector.end(), queue);
	boost::compute::copy(vector.cbegin(), vector.cend(), data.begin(), queue);

	return 0;
}