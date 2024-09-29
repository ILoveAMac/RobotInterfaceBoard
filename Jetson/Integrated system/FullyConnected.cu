//
// Created by Wihan on 2024-09-09.
//

#include "FullyConnected.cuh"

// CUDA Kernel for output layer
// Each thread in the CUDA grid will compute one output neuron (one element of the output vector).
// The computation for each output neuron is independent of the others.
__global__ void fullyConnectedKernel(const __half *input, const __half *weights, const __half *biases, __half *output,
                                     const int inputSize, const int outputSize, const bool applyActivation)
{

    // Get the index of the current thread
    // This index corresponds to a specific output neuron
    const int neuronIndex = blockIdx.x * blockDim.x + threadIdx.x;

    // Check that we remain within the bounds of the output vector
    if (neuronIndex < outputSize)
    {
        // Accumulator for result
        __half sum = 0.0f;

        // Perform the dot product between the input vector and the row of the weights matrix
        for (int i = 0; i < inputSize; i++)
        {
            sum += weights[neuronIndex * inputSize + i] * input[i];
        }

        // Add the bias
        sum += biases[neuronIndex];

        // Only apply the activation if this is an inner layer
        if (applyActivation)
        {
            sum = (sum > 0.0f) ? sum : 0.1f * sum; // Leaky ReLu
        }

        // Write the result to the output
        output[neuronIndex] = sum;
    }
}

FullyConnected::FullyConnected(const int inputSize, const int outputSize, const ModelLoadingHelper &ml,
                               const std::string &layerName, const bool applyActivation) : d_weights(nullptr), d_biases(nullptr), ml(ml)
{
    this->inputSize = inputSize;
    this->outputSize = outputSize;

    this->layerName = layerName;

    this->applyActivation = applyActivation;

    // allocate memory for the intermediate results
    // cudaMallocManaged(&d_intermediate, outputSize * sizeof(__half));
    cudaMalloc(&d_intermediate, outputSize * sizeof(__half));
}

FullyConnected::~FullyConnected()
{
    if (d_weights != nullptr)
    {
        cudaFree(d_weights);
    }

    if (d_biases != nullptr)
    {
        cudaFree(d_biases);
    }
}

void FullyConnected::loadData()
{
    // Load the weights into a vector from a file using the ModelLoadingHelper
    const std::string weightPath = "fcs." + this->layerName + ".weight.bin";
    const auto weights = ml.loadFCL(weightPath);

    const std::string biasPath = "fcs." + this->layerName + ".bias.bin";
    const auto biases = ml.load1D(biasPath);

    // Now vectors must be flattened and assigned to the GPU
    const auto f_weights = flatten2D(weights);
    allocateAndCopyUnifiedMemory(f_weights, d_weights);

    allocateAndCopyUnifiedMemory(biases, d_biases);
}

__half *FullyConnected::forward(const __half *input)
{
    int threadsPerBlock = 8;
    int blocksPerGrid = (outputSize + threadsPerBlock - 1) / threadsPerBlock;

    fullyConnectedKernel<<<blocksPerGrid, threadsPerBlock>>>(input, d_weights, d_biases, d_intermediate, inputSize, outputSize, applyActivation);

    return d_intermediate;
}

std::vector<__half> FullyConnected::flatten2D(const std::vector<std::vector<__half>> &input)
{
    std::vector<__half> flattened;
    for (const auto &row : input)
    {
        flattened.insert(flattened.end(), row.begin(), row.end());
    }
    return flattened;
}

void FullyConnected::allocateAndCopyUnifiedMemory(const std::vector<__half> &flattenedData, __half *&d_ptr)
{
    // Calculate the size of the flattened data in bytes
    const size_t dataSize = flattenedData.size() * sizeof(__half);

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
    }
    // d_ptr now points to the GPU memory containing the flattened data
}

bool FullyConnected::checkCudaError(const cudaError_t err, const char *msg)
{
    if (err != cudaSuccess)
    {
        std::cerr << "CUDA error: " << msg << ": " << cudaGetErrorString(err) << std::endl;
        return false; // Return false to indicate failure
    }
    return true; // Return true if no error
}
