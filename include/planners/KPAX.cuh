#pragma once
#include "planners/Planner.cuh"
#include "graphs/Graph.cuh"

class KPAX : public Planner
{
public:
    /**************************** CONSTRUCTORS ****************************/
    KPAX();

    /****************************    METHODS    ****************************/
    void plan(float* h_initial, float* h_goal, float* d_obstacles_ptr, uint h_obstaclesCount) override;
    void planBench(float* h_initial, float* h_goal, float* d_obstacles_ptr, uint h_obstaclesCount, int benchItr);
    void propagateFrontier(float* d_obstacles_ptr, uint h_obstaclesCount);
    void updateFrontier();
    void writeDeviceVectorsToCSV(int itr);
    void writeExecutionTimeToCSV(double time);

    /****************************    FIELDS    ****************************/
    // --- host fields ---
    Graph graph_;
    uint h_frontierSize_, h_frontierNextSize_, h_activeBlockSize_, h_frontierRepeatSize_, h_propIterations_;
    float h_fAccept_;

    // --- device fields ---
    thrust::device_vector<bool> d_frontier_, d_frontierNext_;
    thrust::device_vector<uint> d_activeFrontierIdxs_, d_frontierScanIdx_, d_activeFrontierRepeatCount_, d_frontierRepeatScanIdx_,
      d_activeFrontierRepeatIdxs_;
    thrust::device_vector<int> d_unexploredSamplesParentIdxs_;
    thrust::device_vector<float> d_unexploredSamples_, d_goalSample_;
    float *d_unexploredSamples_ptr_, *d_goalSample_ptr_;
    bool *d_frontier_ptr_, *d_frontierNext_ptr_;
    uint *d_activeFrontierIdxs_ptr_, *d_frontierScanIdx_ptr_, *d_activeFrontierRepeatCount_ptr_, *d_frontierRepeatScanIdx_ptr_,
      *d_activeFrontierRepeatIdxs_ptr_;
    int* d_unexploredSamplesParentIdxs_ptr_;
};

/**************************** DEVICE FUNCTIONS ****************************/

/***************************/
/* PROPAGATE FRONTIER KERNEL 1 */
/***************************/
// --- Propagates current frontier. Builds new frontier. ---
// --- One Block Per Frontier Sample ---
__global__ void propagateFrontier_kernel1(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples,
                                          uint frontierSize, curandState* randomSeeds, int* unexploredSamplesParentIdxs, float* obstacles,
                                          int obstaclesCount, int* activeSubVertices, float* vertexScores, bool* frontierNext,
                                          int* vertexCounter, int* validVertexCounter, float* minValueInRegion);

__global__ void propagateFrontier_kernel2(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples,
                                          uint frontierSize, curandState* randomSeeds, int* unexploredSamplesParentIdxs, float* obstacles,
                                          int obstaclesCount, int* activeSubVertices, float* vertexScores, bool* frontierNext,
                                          int* vertexCounter, int* validVertexCounter, int iterations, float* minValueInRegion);

__global__ void
updateFrontier_kernel(bool* frontier, bool* frontierNext, uint* activeFrontierNextIdxs, uint frontierNextSize, float* xGoal, int treeSize,
                      float* unexploredSamples, float* treeSamples, int* unexploredSamplesParentIdxs, int* treeSamplesParentIdxs,
                      float* treeSampleCosts, int* pathToGoal, uint* activeFrontierRepeatCount, int* validVertexCounter,
                      curandState* randomSeeds, float* vertexScores, float* controlPathToGoal, float fAccept);