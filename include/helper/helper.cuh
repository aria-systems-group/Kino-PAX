#pragma once

#include <stdio.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <string>
#include <cstdlib>
#include <math.h>
#include <fstream>
#include "cuda.h"
#include "cuda_runtime.h"
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <iomanip>
#include "config/config.h"
#define _USE_MATH_DEFINES

// A macro for checking the error codes of cuda runtime calls
#define CUDA_ERROR_CHECK(expr)                                                \
    {                                                                         \
        cudaError_t err = expr;                                               \
        if(err != cudaSuccess)                                                \
            {                                                                 \
                printf("CUDA call failed!\n\t%s\n", cudaGetErrorString(err)); \
                exit(1);                                                      \
            }                                                                 \
    }

template <typename T>
void printDeviceVector(const T* d_ptr, int size);

__device__ void printSample(float* x, int sampleDim);
std::vector<float> readObstaclesFromCSV(const std::string& filename, int& numObstacles, int workspaceDim);

template <typename T>
void writeVectorToCSV(const thrust::host_vector<T>& vec, const std::string& filename, int rows, int cols);

template <typename T>
void copyAndWriteVectorToCSV(const thrust::device_vector<T>& d_vec, const std::string& filename, int rows, int cols);

// Implement the template functions in the header file

template <typename T>
void printDeviceVector(const T* d_ptr, int size)
{
    thrust::host_vector<T> h_vec(size);
    cudaMemcpy(thrust::raw_pointer_cast(h_vec.data()), d_ptr, size * sizeof(T), cudaMemcpyDeviceToHost);
    for(int i = 0; i < size; ++i)
        {
            std::cout << h_vec[i] << " ";
        }
    std::cout << std::endl;
}

template <typename T>
void writeVectorToCSV(const thrust::host_vector<T>& vec, const std::string& filename, int rows, int cols, bool append = false)
{
    std::ofstream file;
    if(append)
        {
            file.open(filename, std::ios_base::app);  // Open in append mode
        }
    else
        {
            file.open(filename);
        }
    file << std::fixed << std::setprecision(6);

    for(int i = 0; i < rows; i++)
        {
            for(int j = 0; j < cols; j++)
                {
                    file << vec[i * cols + j];
                    if(j < cols - 1)
                        {
                            file << ",";
                        }
                }
            file << std::endl;
        }

    file.close();
}

template <typename T>
void copyAndWriteVectorToCSV(const thrust::device_vector<T>& d_vec, const std::string& filename, int rows, int cols, bool append = false)
{
    thrust::host_vector<T> h_vec(d_vec.size());
    cudaMemcpy(thrust::raw_pointer_cast(h_vec.data()), thrust::raw_pointer_cast(d_vec.data()), d_vec.size() * sizeof(T),
               cudaMemcpyDeviceToHost);
    writeVectorToCSV(h_vec, filename, rows, cols, append);
}

template <typename T>
inline void writeValueToCSV(const T& value, const std::string& filename)
{
    std::ofstream file;
    file.open(filename, std::ios_base::app);  // Open in append mode

    // Set precision for floating-point numbers
    if constexpr(std::is_floating_point_v<std::decay_t<decltype(value)>>)
        {
            file << std::fixed << std::setprecision(10);
        }

    file << value << std::endl;
    file.close();
}

__device__ __forceinline__ float atomicMinFloat(float* addr, float value)
{
    float old;
    old = (value >= 0) ? __int_as_float(atomicMin((int*)addr, __float_as_int(value)))
                       : __uint_as_float(atomicMax((unsigned int*)addr, __float_as_uint(value)));

    return old;
}

#define gpuErrchk(ans)                        \
    {                                         \
        gpuAssert((ans), __FILE__, __LINE__); \
    }
inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true)
{
    if(code != cudaSuccess)
        {
            fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
            if(abort) exit(code);
        }
}

inline int iDivUp(int a, int b)
{
    return (a + b - 1) / b;
}

__device__ __forceinline__ float distance(float* a, float* b)
{
    if(W_DIM == 2)
        {
            return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2));
        }
    else if(W_DIM == 3)
        {
            return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) + pow(a[2] - b[2], 2));
        }
    else
        {
            return -1;
        }
}
