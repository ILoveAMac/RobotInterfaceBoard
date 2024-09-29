//
// Created by Wihan on 2024-09-09.
//

#ifndef FULLYCONNECTED_CUH
#define FULLYCONNECTED_CUH

#include <Layer.cuh>
#include "ModelLoadingHelper.h"
#include <vector>
#include <cstring>
#include <cuda_runtime.h>

class FullyConnected : public Layer
{
public:
    // Constructor
    FullyConnected(int inputSize, int outputSize, const ModelLoadingHelper &ml, const std::string &layerName, bool applyActivation);

    // Destructor to free GPU memory
    ~FullyConnected() override;

    // Function to load weights and biases from binary files
    void loadData() override;

    // Forward function to perform the fully connected operation
    __half *forward(const __half *input) override;

    // Fully connected layers do not modify output dimensions (just the output size)
    int getOutputHeight() const override { return 1; }
    int getOutputWidth() const override { return 1; }
    int getOutputChannels() const override { return outputSize; }

private:
    int inputSize;  // The size of the input vector
    int outputSize; // The size of the output vector

    // Weights and biases on the GPU
    __half *d_weights; // Pointer to device memory for weights
    __half *d_biases;  // Pointer to device memory for biases

    // Model loading helper for loading weights and biases
    ModelLoadingHelper ml;
    std::string layerName;

    // Is this an intermediate layer or not:
    bool applyActivation;

    // Scratch space for storing intermediate results
    __half *d_intermediate;

    static std::vector<__half> flatten2D(const std::vector<std::vector<__half>> &input);

    // Utility functions for GPU memory management
    static void allocateAndCopyUnifiedMemory(const std::vector<__half> &flattenedData, __half *&d_ptr);

    static bool checkCudaError(cudaError_t err, const char *msg);
};

#endif // FULLYCONNECTED_CUH
