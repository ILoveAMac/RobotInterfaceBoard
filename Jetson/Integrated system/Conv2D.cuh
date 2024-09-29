//
// Created by Wihan on 2024-09-08.
//

#ifndef CONV2D_CUH
#define CONV2D_CUH
#include <string>
#include <cstring>
#include <cuda_runtime.h>

#include <cuda_fp16.h>

#include "ModelLoadingHelper.cuh"
#include "Layer.cuh"

class Conv2D : public Layer
{
public:
    Conv2D(int kernelSize, int numFilters, int stride, int padding, const std::string &layerName,
           const ModelLoadingHelper &ml, int outHeight, int outWidth, int outChannels, int inputHeight, int inputWidth,
           int inputChannels);

    // Free allocated GPU memory here
    ~Conv2D() override;

    // Function to load data from binary files
    // Loaded data will be stored in pointers to be used in a cuda kernel
    void loadData() override;

    // Forward function to perform convolution, batch normalization, and activation
    // It takes an input tensor (flattened) and returns the output tensor (flattened)
    __half *forward(const __half *input) override;

    int getOutputHeight() const override;

    int getOutputWidth() const override;

    int getOutputChannels() const override;

private:
    int kernelSize;        // Size of the convolution kernel
    int numFilters;        // Number of filters in the Conv2D layer
    int stride;            // Stride for the convolution operation
    int padding;           // Padding for the convolution
    std::string layerName; // Name of the layer (for identifying the layer or file paths)

    // Input shape
    int inputHeight;
    int inputWidth;
    int inputChannels;

    // Output shape
    int outputHeight, outputWidth, outputChannels;

    // Pointers to hold weights and biases on the GPU
    __half *d_weights; // Pointer to device memory for kernel weights

    // Batch normalization parameters
    __half *d_gamma;       // Pointer to device memory for batch norm scale (gamma)
    __half *d_beta;        // Pointer to device memory for batch norm shift (beta)
    __half *d_runningMean; // Pointer to device memory for running mean
    __half *d_runningVar;  // Pointer to device memory for running variance

    // Model Loading helper to load the data for the model during init
    ModelLoadingHelper ml;

    // Scratch space for storing intermediate results
    __half *d_intermediate; // Pointer to device memory for intermediate results

    static std::vector<__half> flatten2D(const std::vector<std::vector<__half>> &input);

    static std::vector<__half> flatten4D(const std::vector<std::vector<std::vector<std::vector<__half>>>> &input);

    static void allocateAndCopyUnifiedMemory(const std::vector<__half> &flattenedData, __half *&d_ptr);

    static bool checkCudaError(cudaError_t err, const char *msg);
};

#endif // CONV2D_CUH
