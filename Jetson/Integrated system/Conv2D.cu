//
// Created by Wihan on 2024-09-08.
//

#include "Conv2D.cuh"
#include <cuda_runtime.h>
#include <cmath>

// CUDA kernel for Conv2D forward pass with batch normalization and ReLU
__global__ void conv2dForwardKernel(const float *input, float *output, const float *weights, const float *gamma,
                                    const float *beta, const float *runningMean, const float *runningVar,
                                    const int inputHeight, const int inputWidth, const int inputChannels,
                                    const int kernelSize,
                                    const int stride, const int padding, const int outputHeight,
                                    const int outputWidth)
{
    // Get the index of the thread in the output tensor
    const int filter = blockIdx.z;                       // Channel index
    const int h = blockIdx.y * blockDim.y + threadIdx.y; // Output height index
    const int w = blockIdx.x * blockDim.x + threadIdx.x; // Output width index

    // Where are we now:
    // We are a specific filter given by the filter index
    // On the input image we are at position (h, w).
    // (h, w) is the top left corner of the area where we will preform convolution
    // Now, we must preform the convolution operation at position (h, w) using the specific filter
    // The size of the filter is given by kernelSize, and we use it to ensure we preform the convolution within
    // the bounds of the kernel

    // Additional things to take not of:
    // We are using padding. The padding will be used to keep the input the same size as the output.
    // We are using a stride. The stride will affect the output size

    // Check that we are within bounds of the output tensor
    if (h < outputHeight && w < outputWidth)
    {
        // Accumulator for result
        float sum = 0.0f;

        // For every input channel
        for (int inputChannel = 0; inputChannel < inputChannels; inputChannel++)
        {
            // Two for loops to move over the kernel, the kernel is always square
            for (int kh = 0; kh < kernelSize; kh++)
            {
                for (int kw = 0; kw < kernelSize; kw++)
                {
                    // Calculate our position on the input tensor
                    // kh and kw are the row and column offsets within the kernel
                    const int inputH = h * stride - padding + kh;
                    const int inputW = w * stride - padding + kw;

                    if (inputH >= 0 && inputH < inputHeight && inputW >= 0 && inputW < inputWidth)
                    {
                        // Calculate the index in the flattened weights array
                        const int weightIndex = ((filter * inputChannels + inputChannel) * kernelSize * kernelSize) + (kh * kernelSize + kw);
                        sum += input[(inputChannel * inputHeight + inputH) * inputWidth + inputW] * weights[weightIndex];
                    }
                    // else the bounds are invalid,
                    // thus we treat this as padding with zeros and do not add anything to the sum
                }
            }
        }

        // Apply batch normalization
        const float mean = runningMean[filter];
        const float var = runningVar[filter];
        const float gammaVal = gamma[filter];
        const float betaVal = beta[filter];
        sum = gammaVal * (sum - mean) / sqrtf(var + 1e-8) + betaVal;

        // Apply LeRelu
        sum = (sum > 0.0f) ? sum : 0.1f * sum;

        output[(filter * outputHeight + h) * outputWidth + w] = sum;
    }
}

// CUDA kernel for Conv2D forward pass with batch normalization and Leaky ReLU using shared memory
__global__ void conv2dForwardKernelShared(const float *input, float *output, const float *weights, const float *gamma,
                                          const float *beta, const float *runningMean, const float *runningVar,
                                          const int inputHeight, const int inputWidth, const int inputChannels,
                                          const int kernelSize,
                                          const int stride, const int padding, const int outputHeight,
                                          const int outputWidth)
{
    // Calculate output coordinates
    const int filter = blockIdx.z;                           // Output channel (filter) index
    const int h_out = blockIdx.y * blockDim.y + threadIdx.y; // Output height index
    const int w_out = blockIdx.x * blockDim.x + threadIdx.x; // Output width index

    // Check if within output bounds
    if (h_out >= outputHeight || w_out >= outputWidth)
        return;

    // Calculate the corresponding input coordinates
    const int h_in = h_out * stride - padding;
    const int w_in = w_out * stride - padding;

    // Shared memory for input tile
    extern __shared__ float sharedInput[];
    // Size of shared memory per input channel
    const int sharedMemPerChannel = (blockDim.y + kernelSize - 1) * (blockDim.x + kernelSize - 1);

    // Load input tile into shared memory for each input channel
    for (int c = 0; c < inputChannels; ++c)
    {
        for (int i = threadIdx.y; i < blockDim.y + kernelSize - 1; i += blockDim.y)
        {
            for (int j = threadIdx.x; j < blockDim.x + kernelSize - 1; j += blockDim.x)
            {
                int h = h_in + i;
                int w = w_in + j;
                float value = 0.0f;
                if (h >= 0 && h < inputHeight && w >= 0 && w < inputWidth)
                {
                    int inputIdx = (c * inputHeight + h) * inputWidth + w;
                    value = input[inputIdx];
                }
                int sharedIdx = c * sharedMemPerChannel + i * (blockDim.x + kernelSize - 1) + j;
                sharedInput[sharedIdx] = value;
            }
        }
    }

    // Synchronize to ensure all data is loaded into shared memory
    __syncthreads();

    // Perform convolution
    float sum = 0.0f;
    for (int c = 0; c < inputChannels; ++c)
    {
        for (int kh = 0; kh < kernelSize; ++kh)
        {
            for (int kw = 0; kw < kernelSize; ++kw)
            {
                int sharedIdx = c * sharedMemPerChannel + (threadIdx.y + kh) * (blockDim.x + kernelSize - 1) + (threadIdx.x + kw);
                int weightIdx = ((filter * inputChannels + c) * kernelSize * kernelSize) + (kh * kernelSize + kw);
                sum += sharedInput[sharedIdx] * weights[weightIdx];
            }
        }
    }

    // Apply batch normalization
    const float mean = runningMean[filter];
    const float var = runningVar[filter];
    const float gammaVal = gamma[filter];
    const float betaVal = beta[filter];
    sum = gammaVal * (sum - mean) / sqrtf(var + 1e-8f) + betaVal;

    // Apply Leaky ReLU
    sum = (sum > 0.0f) ? sum : 0.1f * sum;

    // Write the result to the output tensor
    int outputIdx = (filter * outputHeight + h_out) * outputWidth + w_out;
    output[outputIdx] = sum;
}

Conv2D::Conv2D(const int kernelSize, const int numFilters, const int stride, const int padding,
               const std::string &layerName, const ModelLoadingHelper &ml, const int outHeight, const int outWidth,
               const int outChannels, const int inputHeight, const int inputWidth,
               const int inputChannels) : d_weights(nullptr),
                                          d_gamma(nullptr),
                                          d_beta(nullptr),
                                          d_runningMean(nullptr),
                                          d_runningVar(nullptr), ml(ml)
{
    this->kernelSize = kernelSize;
    this->numFilters = numFilters;
    this->stride = stride;
    this->padding = padding;
    this->layerName = layerName;

    // Output shape
    this->outputChannels = outChannels;
    this->outputHeight = outHeight;
    this->outputWidth = outWidth;

    // Input shape
    this->inputHeight = inputHeight;
    this->inputWidth = inputWidth;
    this->inputChannels = inputChannels;

    // Allocate scratch space for the output tensor
    // cudaMallocManaged(&d_intermediate, outputHeight * outputWidth * numFilters * sizeof(float));
    cudaMalloc(&d_intermediate, outputHeight * outputWidth * numFilters * sizeof(float));
}

Conv2D::~Conv2D()
{
    // Check if the pointers are not null and free GPU memory
    if (d_weights != nullptr)
    {
        cudaFree(d_weights);
        d_weights = nullptr; // Set the pointer to nullptr after freeing
    }

    if (d_gamma != nullptr)
    {
        cudaFree(d_gamma);
        d_gamma = nullptr;
    }

    if (d_beta != nullptr)
    {
        cudaFree(d_beta);
        d_beta = nullptr;
    }

    if (d_runningMean != nullptr)
    {
        cudaFree(d_runningMean);
        d_runningMean = nullptr;
    }

    if (d_runningVar != nullptr)
    {
        cudaFree(d_runningVar);
        d_runningVar = nullptr;
    }
}

// This function will load all data associated with the conv layer into vectors
// based on the name of the vector and then flatten the vectors and move them to the GPU
void Conv2D::loadData()
{
    // Load the weights into a vector from a file using the ModelLoadingHelper
    const std::string convWeightPath = "darknet." + this->layerName + ".conv.weight.bin";
    const auto weights = ml.loadConv4D(convWeightPath);

    // Load the batch norm weights
    const std::string batchNormWeights = "darknet." + this->layerName + ".batchnorm.weight.bin";
    const auto bnWeights = ml.load1D(batchNormWeights);

    // Load the batch norm bias
    const std::string batchNormBias = "darknet." + this->layerName + ".batchnorm.bias.bin";
    const auto bnBias = ml.load1D(batchNormBias);

    // Load the batch norm running mean
    const std::string batchRunningMean = "darknet." + this->layerName + ".batchnorm.running_mean.bin";
    const auto bnRunningMean = ml.load1D(batchRunningMean);

    // Load the batch norm running var
    const std::string batchRunningVar = "darknet." + this->layerName + ".batchnorm.running_var.bin";
    const auto bnRunningVar = ml.load1D(batchRunningVar);

    // Now vectors must be flattened and assigned to the GPU

    // Weights
    const auto f_weights = flatten4D(weights);
    allocateAndCopyUnifiedMemory(f_weights, d_weights);

    // bnWeights
    allocateAndCopyUnifiedMemory(bnWeights, d_gamma);

    // bnBias
    allocateAndCopyUnifiedMemory(bnBias, d_beta);

    // bnRunningMean
    allocateAndCopyUnifiedMemory(bnRunningMean, d_runningMean);

    // bnRunningVar
    allocateAndCopyUnifiedMemory(bnRunningVar, d_runningVar);
}

float *Conv2D::forward(const float *input)
{
    // Block and grid sizes to launch the CUDA Kernel
    dim3 blockDim(16, 16); // 16x16 threads per block (256 which is dividable by 32 as warps run in groups of 32)
    // Calculation of the grid dimensions below ensures that we always have enough blocks to cover the whole image
    dim3 gridDim((outputWidth + blockDim.x - 1) / blockDim.x, (outputHeight + blockDim.y - 1) / blockDim.y, numFilters);

    // Launch the CUDA Kernel
    // conv2dForwardKernel<<<gridDim, blockDim>>>(input, d_intermediate, d_weights, d_gamma, d_beta, d_runningMean, d_runningVar,
    //                                           inputHeight, inputWidth, inputChannels, kernelSize, stride, padding,
    //                                           outputHeight, outputWidth);

    // Calculate shared memory size
    int sharedInputSizePerChannel = (blockDim.y + kernelSize - 1) * (blockDim.x + kernelSize - 1);
    size_t sharedMemorySize = inputChannels * sharedInputSizePerChannel * sizeof(float);

    // Launch the kernel
    conv2dForwardKernelShared<<<gridDim, blockDim, sharedMemorySize>>>(
        input, d_intermediate, d_weights, d_gamma, d_beta, d_runningMean, d_runningVar,
        inputHeight, inputWidth, inputChannels, kernelSize, stride, padding,
        outputHeight, outputWidth);

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
        std::cerr << "CUDA error: " << cudaGetErrorString(err) << std::endl;
    }

    return d_intermediate;
}

int Conv2D::getOutputHeight() const
{
    return this->outputHeight;
}

int Conv2D::getOutputWidth() const
{
    return this->outputWidth;
}

int Conv2D::getOutputChannels() const
{
    return this->outputChannels;
}

std::vector<float> Conv2D::flatten2D(const std::vector<std::vector<float>> &input)
{
    std::vector<float> flattened;
    for (const auto &row : input)
    {
        flattened.insert(flattened.end(), row.begin(), row.end());
    }
    return flattened;
}

std::vector<float> Conv2D::flatten4D(const std::vector<std::vector<std::vector<std::vector<float>>>> &input)
{
    std::vector<float> flattened;
    for (const auto &tensor : input)
    {
        for (const auto &matrix : tensor)
        {
            for (const auto &row : matrix)
            {
                flattened.insert(flattened.end(), row.begin(), row.end());
            }
        }
    }
    return flattened;
}

void Conv2D::allocateAndCopyUnifiedMemory(const std::vector<float> &flattenedData, float *&d_ptr)
{
    // Calculate the size of the flattened data in bytes
    const size_t dataSize = flattenedData.size() * sizeof(float);

    // Step 1: Allocate GPU memory
    const cudaError_t err = cudaMalloc(&d_ptr, dataSize); // Allocate memory on the GPU (device)
    if (!checkCudaError(err, "Failed to allocate GPU memory"))
    {
        return; // Handle error: early exit if allocation failed
    }

    // Step 2: Copy data from CPU to GPU
    const cudaError_t memcpyErr = cudaMemcpy(d_ptr, flattenedData.data(), dataSize, cudaMemcpyHostToDevice);
    if (!checkCudaError(memcpyErr, "Failed to copy data to GPU memory"))
    {
        cudaFree(d_ptr); // Free GPU memory if copying fails
        return;          // Handle error: early exit if memcpy failed
    }
    // d_ptr now points to the GPU memory containing the flattened data
}

bool Conv2D::checkCudaError(const cudaError_t err, const char *msg)
{
    if (err != cudaSuccess)
    {
        std::cerr << "CUDA error: " << msg << ": " << cudaGetErrorString(err) << std::endl;
        return false; // Return false to indicate failure
    }
    return true; // Return true if no error
}
