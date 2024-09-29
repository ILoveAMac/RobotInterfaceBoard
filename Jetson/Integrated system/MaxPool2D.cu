//
// Created by Wihan on 2024-09-09.
//

#include "MaxPool2D.cuh"

__global__ void maxPool2DForwardKernel(const __half *input, __half *output,
                                       const int inputHeight, const int inputWidth, int inputChannels,
                                       const int outputHeight, const int outputWidth, const int kernelSize,
                                       const int stride)
{

    // Get the index of the thread in the output tensor
    int c = blockIdx.z;                            // Channel index
    int h = blockIdx.y * blockDim.y + threadIdx.y; // Output height index
    int w = blockIdx.x * blockDim.x + threadIdx.x; // Output width index

    // Where are we now:
    // We are at a specific channel in the input tensor given by c
    // We are at pixel coordinates (h, w) in the tensor at channel c
    // The coordinates point to the top left corner of the box where we have to preform max pooling

    // Check that the thread is within bounds of the output tensor
    if (h < outputHeight && w < outputWidth)
    {
        // We initialize the max value to a very low number
        __half maxVal = __float2half(-10000.0f); // -10000 should be low enough

        // Two for loops to move the max pooling window across the input
        for (int kh = 0; kh < kernelSize; kh++)
        {
            for (int kw = 0; kw < kernelSize; kw++)
            {
                // Calculate the position in the input tensor
                const int inputH = h * stride + kh;
                const int inputW = w * stride + kw;

                // Check that we are within the bounds of the input tensor
                if (inputH < inputHeight && inputW < inputWidth)
                {
                    // Calculate the index in the input tensor
                    const int inputIndex = (c * inputHeight + inputH) * inputWidth + inputW;
                    maxVal = (maxVal > input[inputIndex]) ? maxVal : input[inputIndex];
                }
            }
        }

        // Now we store the maximum value in the output tensor at the appropriate index
        const int outputIndex = (c * outputHeight + h) * outputWidth + w;
        output[outputIndex] = maxVal;
    }
}

MaxPool2D::MaxPool2D(const int inputHeight, const int inputWidth, const int inputChannels, const int outputHeight,
                     const int outputWidth,
                     const int outputChannels)
{
    this->inputHeight = inputHeight;
    this->inputWidth = inputWidth;
    this->inputChannels = inputChannels;

    this->outputHeight = outputHeight;
    this->outputWidth = outputWidth;
    this->outputChannels = outputChannels;

    // allcoate memory for the intermediate results
    // cudaMallocManaged(&d_intermediate, outputHeight * outputWidth * inputChannels * sizeof(__half));
    cudaMalloc(&d_intermediate, outputHeight * outputWidth * inputChannels * sizeof(__half));
}

MaxPool2D::~MaxPool2D()
{
    // There is no memory to free for the max pooling layer
}

__half *MaxPool2D::forward(const __half *input)
{
    // Define block and grid sizes for the CUDA Kernel
    dim3 blockDim(8, 16); // 16x16 threads per block
    dim3 gridDim((outputWidth + blockDim.x - 1) / blockDim.x, (outputHeight + blockDim.y - 1) / blockDim.y, inputChannels);

    // Launch the Max Pooling kernel
    maxPool2DForwardKernel<<<gridDim, blockDim>>>(input, d_intermediate,
                                                  inputHeight, inputWidth, inputChannels,
                                                  outputHeight, outputWidth, kernelSize, stride);

    return d_intermediate;
}

int MaxPool2D::getOutputHeight() const
{
    return this->outputHeight;
}

int MaxPool2D::getOutputWidth() const
{
    return this->outputWidth;
}

int MaxPool2D::getOutputChannels() const
{
    return this->outputChannels;
}
