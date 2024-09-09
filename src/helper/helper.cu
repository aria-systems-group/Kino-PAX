#include "helper/helper.cuh"

__device__ void printSample(float* x, int sampleDim)
{
    for(int i = 0; i < sampleDim; ++i)
        {
            printf("%f ", x[i]);
        }
    printf("\n");
}

// Function to read obstacles from a CSV file
std::vector<float> readObstaclesFromCSV(const std::string& filename, int& numObstacles, int workspaceDim)
{
    std::vector<float> obstacles;
    std::ifstream file(filename);

    if(!file.is_open())
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            exit(1);
        }

    std::string line;
    while(std::getline(file, line))
        {
            std::stringstream ss(line);
            float value;
            while(ss >> value)
                {
                    obstacles.push_back(value);
                    if(ss.peek() == ',') ss.ignore();
                }
        }

    file.close();
    numObstacles = obstacles.size() / (2 * workspaceDim);
    return obstacles;
}
