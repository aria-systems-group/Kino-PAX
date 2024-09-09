#include "collisionCheck/collisionCheck.cuh"

__device__ bool isBroadPhaseValid(float *bbMin, float *bbMax, float *obs)
{
    for(int d = 0; d < W_DIM; ++d)
        {
            if(bbMax[d] <= obs[d] || obs[W_DIM + d] <= bbMin[d]) return true;
        }
    return false;
}

__device__ bool isFinePhaseValid(float *x0, float *x1, float *obs)
{
    float x0_to_x1[W_DIM];

    for(int d = 0; d < W_DIM; ++d)
        {
            float lambda;
            x0_to_x1[d] = x1[d] - x0[d];
            if(x0[d] < obs[d])
                {
                    lambda = (obs[d] - x0[d]) / x0_to_x1[d];
                }
            else
                {
                    lambda = (obs[W_DIM + d] - x0[d]) / x0_to_x1[d];
                }
            if(faceContainsProjection(x0, x1, lambda, d, obs)) return false;
        }
    return true;
}

__device__ bool faceContainsProjection(float *x0, float *x1, float lambda, int j, float *obs)
{
    for(int d = 0; d < W_DIM; ++d)
        {
            float projection = x0[d] + (x1[d] - x0[d]) * lambda;
            if(d != j && !(obs[d] <= projection && projection <= obs[W_DIM + d]))
                {
                    if(!isFinePhaseValid(x0, x1, obs))
                        {
                            return false;
                        }
                }
        }
    return true;
}

__device__ bool isMotionValid(float *x0, float *x1, float *bbMin, float *bbMax, float *obstacles, int obstaclesCount)
{
    for(int obsIdx = 0; obsIdx < obstaclesCount; ++obsIdx)
        {
            float obs[2 * W_DIM];
            for(int d = 0; d < W_DIM; ++d)
                {
                    obs[d]         = obstacles[obsIdx * 2 * W_DIM + d];
                    obs[W_DIM + d] = obstacles[obsIdx * 2 * W_DIM + W_DIM + d];
                }
            if(!isBroadPhaseValid(bbMin, bbMax, obs)) return false;
        }
    return true;
}
