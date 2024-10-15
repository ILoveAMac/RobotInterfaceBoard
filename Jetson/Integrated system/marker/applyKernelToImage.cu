//
// Created by Wihan on 2024-09-24.
//

#include "applyKernelToImage.cuh"

__global__ void applyKernelToImageKernel(const float *inputImg, float *outputImg, const float *kernel,
                                         const int inputWidth, const int inputHeight,
                                         const int kernelWidth, const int kernelHeight,
                                         const int padding) {
    // Calculate the position of the current thread in the output image
    const int h = blockIdx.y * blockDim.y + threadIdx.y;
    const int w = blockIdx.x * blockDim.x + threadIdx.x;

    // Check if we are within the bounds of the output image
    // We are assuming a stride of 1 so we use the input image dimensions as the input and output dimensions will match
    if (h < inputHeight && w < inputWidth) {
        float sum = 0.0f;

        // Perform convolution operation with "same" padding
        for (int kh = 0; kh < kernelHeight; ++kh) {
            for (int kw = 0; kw < kernelWidth; ++kw) {
                // Calculate the corresponding position in the input image with stride = 1
                const int inputH = h - padding + kh;
                const int inputW = w - padding + kw;

                // Only add to the sum if we're within valid input image bounds
                if (inputH >= 0 && inputH < inputHeight && inputW >= 0 && inputW < inputWidth) {
                    sum += inputImg[inputH * inputWidth + inputW] * kernel[kh * kernelWidth + kw];
                }
                // Else, it's treated as zero due to "same" padding
            }
        }

        // Store the result in the output image
        outputImg[h * inputWidth + w] = sum;
    }
}

applyKernelToImage::applyKernelToImage(const int width, const int height, const int kernelWidth, const int kernelHeight, float *kernel) {
    this->inputHeight = height;
    this->inputWidth = width;
    this->kernelWidth = kernelWidth;
    this->kernelHeight = kernelHeight;

    this->kernel = kernel;

    // Allocate memory for output image
    cudaMalloc(&this->output, sizeof(float) * this->inputWidth * this->inputHeight);
}

applyKernelToImage::~applyKernelToImage() {
    // Free GPU memory
    cudaFree(this->kernel);
    cudaFree(this->output);
}

float * applyKernelToImage::apply(const float *inputImg) const {
    // Calculate the padding needed for "same" padding
    const int padding = (kernelHeight - 1) / 2; // Images are in format x*x so the padding in both dimensions are the same

    // Define the block and grid sizes
    dim3 blockSize(16, 16); // 16x16 threads per block
    dim3 gridSize((inputWidth + blockSize.x - 1) / blockSize.x, (inputHeight + blockSize.y - 1) / blockSize.y);

    // Launch the kernel with "same" padding and stride = 1
    applyKernelToImageKernel<<<gridSize, blockSize>>>(inputImg, this->output, this->kernel,
                                                     inputWidth, inputHeight,
                                                     kernelWidth, kernelHeight,
                                                     padding);

    // Synchronize to ensure the kernel has finished
    cudaDeviceSynchronize();

    // Return pointer to the output image on the GPU
    return this->output;
}
