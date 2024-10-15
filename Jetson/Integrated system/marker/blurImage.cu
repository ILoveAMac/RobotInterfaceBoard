//
// Created by Wihan on 2024-09-24.
//

#include "blurImage.cuh"
#include "applyKernelToImage.cuh"
#include <cmath>
#include <cuda_runtime.h>
#include <iostream>

#define PI 3.141592653589793238462643383279502884197

blurImage::blurImage(const int width, const int height, const int kernelSize, const float sigma)
    : inputWidth(width), inputHeight(height), kernelSize(kernelSize) {
    // Compute the Gaussian kernel
    const float* h_kernel = computeGaussianKernel(sigma, kernelSize);

    // Allocate kernel on the GPU
    cudaMalloc(&d_kernel, sizeof(float) * kernelSize * kernelSize);
    cudaMemcpy(d_kernel, h_kernel, sizeof(float) * kernelSize * kernelSize, cudaMemcpyHostToDevice);

    // Create an instance of applyKernelToImage
    kernelApplier = new applyKernelToImage(width, height, kernelSize, kernelSize, d_kernel);

    // Clean up host memory for the kernel
    delete[] h_kernel;
}

blurImage::~blurImage() {
    // Free GPU memory for the kernel
    cudaFree(d_kernel);

    // Clean up the kernel applier
    delete kernelApplier;
}

float* blurImage::blur(const float* image, int width, int height) const {
    // Delegate the blurring operation to the applyKernelToImage instance
    return kernelApplier->apply(image);
}

float* blurImage::computeGaussianKernel(const float sigma, const int kernelSize) {
    const auto kernel = new float[kernelSize * kernelSize];
    float sum = 0.0f;
    const int halfSize = kernelSize / 2;
    const float sigmaSquared = 2 * sigma * sigma;

    for (int y = -halfSize; y <= halfSize; ++y) {
        for (int x = -halfSize; x <= halfSize; ++x) {
            const float exponent = -(x * x + y * y) / sigmaSquared;
            const float value = expf(exponent) / (PI * sigmaSquared);
            kernel[(y + halfSize) * kernelSize + (x + halfSize)] = value;
            sum += value;
        }
    }

    // Normalize the kernel
    for (int i = 0; i < kernelSize * kernelSize; ++i) {
        kernel[i] /= sum;
    }

    return kernel;
}
