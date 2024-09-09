#pragma once
#include <stdio.h>
#include "config/config.h"
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/count.h>
#include <thrust/unique.h>
#include <thrust/iterator/constant_iterator.h>
#include <curand_kernel.h>
#include "helper/helper.cuh"
#include "statePropagator/statePropagator.cuh"
#include <filesystem>
#include <tuple>
#include <math.h>

class Planner
{
public:
    /**************************** CONSTRUCTORS ****************************/
    Planner();

    /****************************    METHODS    ****************************/
    virtual void plan(float* h_initial, float* h_goal, float* d_obstacles_ptr, uint h_obstaclesCount) = 0;
    void initializeRandomSeeds(int seed);

    /****************************    FIELDS    ****************************/
    // --- host fields ---
    uint h_treeSize_ = 0, h_itr_ = 0, h_blockSize_ = 128, h_gridSize_;
    float h_costToGoal_ = 0.0;
    int h_pathToGoal_;
    float* h_controlPathToGoal_;

    // --- device fields ---
    thrust::device_vector<float> d_treeSamples_, d_treeSampleCosts_, d_controlPathToGoal_;
    thrust::device_vector<int> d_treeSamplesParentIdxs_;

    float *d_treeSamples_ptr_, *d_treeSampleCosts_ptr_, *d_costToGoal_ptr_, *d_controlPathToGoal_ptr_;
    int *d_treeSamplesParentIdxs_ptr_, *d_pathToGoal_ptr_;

    curandState* d_randomSeeds_ptr_;
};

/**************************** DEVICE FUNCTIONS ****************************/

/***************************/
/* INIT RANDOM SEEDS KERNEL */
/***************************/
// --- used to generate random values when propagating frontier on GPU. ---
__global__ void initializeRandomSeeds_kernel(curandState* randomSeeds, int numSeeds, int seed);

/***************************/
/* FIND INDICES BOOL KERNEL */
/***************************/
// --- Finds active indices in a boolean array. ---
__global__ void findInd(uint numSamples, bool* S, uint* scanIdx, uint* activeS);

/***************************/
/* FIND INDICES INT KERNEL */
/***************************/
// --- Finds active indices in an integer array. ---
__global__ void findInd(uint numSamples, uint* S, uint* scanIdx, uint* activeS);

/***************************/
/* REPEAT INDICES KERNEL */
/***************************/
__global__ void repeatInd(uint numSamples, uint* activeS, uint* C, uint* prefixSum, uint* repeatedInd);