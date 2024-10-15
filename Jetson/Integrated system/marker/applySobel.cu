//
// Created by Wihan on 2024-09-24.
//

#include "applySobel.cuh"

__global__ void arcTan2dKernel(const float* G_x, const float* G_y, float* angle, const int width, const int height) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    // Check if the thread is within image bounds
    if (x < width && y < height) {
        const int index = y * width + x;

        // Compute the angle using atan2
        angle[index] = atan2f(G_y[index], G_x[index]);
    }
}

__global__ void computeMagnitudeKernel(const float *G_x, const float *G_y, float *magnitude,
                                       const int width, const int height) {
    // Calculate the global thread coordinates
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    // Ensure the thread is within the image bounds
    if (x < width && y < height) {
        const int idx = y * width + x;
        const float gx = G_x[idx];
        const float gy = G_y[idx];
        magnitude[idx] = sqrtf(gx * gx + gy * gy);
    }
}

applySobel::applySobel(const int width, const int height) : width(width), height(height) {
    // Define the 3x3 Sobel kernels
    constexpr float h_sobel_x[9] = {
        -1, 0, 1,
        -2, 0, 2,
        -1, 0, 1
    };

    constexpr float h_sobel_y[9] = {
        -1, -2, -1,
        0,  0,  0,
        1,  2,  1
    };

    // Allocate memory on the GPU for the Sobel kernels
    cudaMalloc(&sobel_x, sizeof(float) * 9);
    cudaMemcpy(sobel_x, h_sobel_x, sizeof(float) * 9, cudaMemcpyHostToDevice);

    cudaMalloc(&sobel_y, sizeof(float) * 9);
    cudaMemcpy(sobel_y, h_sobel_y, sizeof(float) * 9, cudaMemcpyHostToDevice);

    // Initialize the applyKernelToImage objects for X and Y convolutions
    kernelApplierX = new applyKernelToImage(width, height, 3, 3, sobel_x); // 3x3 kernel for Sobel X
    kernelApplierY = new applyKernelToImage(width, height, 3, 3, sobel_y); // 3x3 kernel for Sobel Y
}

applySobel::~applySobel() {
    // Free GPU memory for the Sobel kernels
    cudaFree(sobel_x);
    cudaFree(sobel_y);

    // Clean up the kernel applier objects
    if (kernelApplierX != nullptr) {
        delete kernelApplierX;
    }
    if (kernelApplierY != nullptr) {
        delete kernelApplierY;
    }
}

std::pmr::vector<float*> applySobel::apply(const float* inputImg) const {
    // Apply the Sobel X kernel to the input image
    const float* G_x = kernelApplierX->apply(inputImg);

    // Apply the Sobel Y kernel to the input image
    const float* G_y = kernelApplierY->apply(inputImg);

    // Calculate the gradient magnitude using G_x and G_y
    float* d_gradientMagnitude = sqrt2D(G_x, G_y);

    // Calculate the gradient direction (angle) using G_x and G_y
    float* d_gradientAngle = arcTan2d(G_x, G_y);

    // Allocate memory on the CPU for the gradient magnitude and angle
    const auto gradientMagnitude = new float[width * height];
    const auto gradientAngle = new float[width * height];

    // Copy data from the GPU to the CPU
    cudaMemcpy(gradientMagnitude, d_gradientMagnitude, sizeof(float) * width * height, cudaMemcpyDeviceToHost);
    cudaMemcpy(gradientAngle, d_gradientAngle, sizeof(float) * width * height, cudaMemcpyDeviceToHost);

    // Free the GPU memory as it's no longer needed
    cudaFree(d_gradientMagnitude);
    cudaFree(d_gradientAngle);

    // Create a std::pmr::vector to store the pointers to gradientMagnitude and gradientAngle
    std::pmr::vector<float*> results{std::pmr::get_default_resource()}; // Use default memory resource
    results.reserve(2); // Reserve space for two elements

    // Add the pointers to the vector
    results.push_back(gradientMagnitude);
    results.push_back(gradientAngle);

    return results;
}


float * applySobel::arcTan2d(const float *G_x, const float *G_y) const {
    // Allocate memory for the angle output on the GPU
    float* d_angle;
    cudaMalloc(&d_angle, sizeof(float) * width * height);

    // Define block and grid sizes
    dim3 blockSize(16, 16); // 16x16 threads per block
    dim3 gridSize((width + blockSize.x - 1) / blockSize.x, (height + blockSize.y - 1) / blockSize.y);

    // Launch the kernel to compute the angle
    arcTan2dKernel<<<gridSize, blockSize>>>(G_x, G_y, d_angle, width, height);
    cudaDeviceSynchronize(); // Wait for the kernel to complete

    // Return the pointer to the angle data on the GPU
    return d_angle;

}

float * applySobel::sqrt2D(const float *G_x, const float *G_y) const {
    // Allocate memory on the GPU for the magnitude image
    float* d_magnitudeImg;
    cudaMalloc(&d_magnitudeImg, sizeof(float) * width * height);

    // Define the block and grid sizes
    dim3 blockSize(16, 16); // 16x16 threads per block
    dim3 gridSize((width + blockSize.x - 1) / blockSize.x,
                  (height + blockSize.y - 1) / blockSize.y);

    // Launch the magnitude computation kernel
    computeMagnitudeKernel<<<gridSize, blockSize>>>(G_x, G_y, d_magnitudeImg, width, height);

    // Synchronize to ensure the kernel has finished
    cudaDeviceSynchronize();

    return d_magnitudeImg;
}
