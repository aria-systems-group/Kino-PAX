#pragma once
#include <curand_kernel.h>
#include "helper/helper.cuh"
#include "config/config.h"
#include "collisionCheck/collisionCheck.cuh"

__device__ bool propagateAndCheck(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount);
__device__ bool propagateAndCheckUnicycle(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount);
__device__ bool propagateAndCheckDoubleIntRungeKutta(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount);
__device__ bool propagateAndCheckDubinsAirplaneRungeKutta(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount);
__device__ bool propagateAndCheckQuadRungeKutta(float* x0, float* x1, curandState* seed, float* obstacles, int obstaclesCount);
__device__ void ode(float* x0dot, float* x0, float* h, float Zc, float Lc, float Mc, float Nc, int itr);

typedef bool (*PropagateAndCheckFunc)(float*, float*, curandState*, float*, int);

/***************************/
/* GET PROPAGATION FUNCTION */
/***************************/
// --- Determins which dynamic model to use. ---
__device__ PropagateAndCheckFunc getPropagateAndCheckFunc();