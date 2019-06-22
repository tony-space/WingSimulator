#include <vector>
#include <iostream>
#include <boost/range.hpp>
#include <boost/range/adaptors.hpp>

int main(int argc, char** argv)
{
	std::vector<int> numbers = { 0,1,2,3,4,5 };
	auto range = numbers | boost::adaptors::transformed([](auto x) {return x * x; });
	for (auto x : range)
		std::cout << x << std::endl;

	return 0;
}