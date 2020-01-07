#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

#include <sstream>

#include <stdexcept>

namespace wing2d
{
	namespace simulation
	{
		namespace cuda
		{
			constexpr unsigned kBlockSize = 64;
			
			inline unsigned GridSize(size_t elements, unsigned blockSize)
			{
				return (unsigned(elements) - 1) / blockSize + 1;
			}

			inline void __cudaSafeCall(cudaError err, const char* function, const char* file, const int line)
			{
				if (err != cudaSuccess)
				{
					std::stringstream errorStream;
					errorStream << "CUDA error occured at " << function << ' ' << file << ':' << line << ' ' << cudaGetErrorString(err);
					throw std::runtime_error(errorStream.str());
				}
			}

			inline void __cudaCheckError(const char* function, const char* file, const int line)
			{
				cudaError err = cudaGetLastError();
				__cudaSafeCall(err, function, file, line);
			}
		}
	}
}

#define CudaSafeCall(err) wing2d::simulation::cuda::__cudaSafeCall(err, __FUNCTION__, __FILE__, __LINE__ )
#define CudaCheckError()  wing2d::simulation::cuda::__cudaCheckError(__FUNCTION__, __FILE__, __LINE__ )