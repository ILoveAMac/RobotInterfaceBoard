//
// Created by Wihan on 2024-09-16.
//

#include "imageToGreyConverter.cuh"

// CUDA Kernel to convert the input image to greyscale
// input image is assumed to be in flattened row major form with shape (h, w, 3)
__global__ void toGreyKernel(const float *inputImg, float *outputImg, const int width, const int height,
                             const float rFactor, const float gFactor, const float bFactor) {
    const int h = blockIdx.y * blockDim.y + threadIdx.y;
    const int w = blockIdx.x * blockDim.x + threadIdx.x;

    // Where are we now:
    // (h, w) is the position where we must preform the grey scaling operation:
    // gray = rFactor * r + gFactor * g + bFactor * b

    // Check that we are within the bounds of the output image
    if (h < height && w < width) {
        // Pixel index
        const int pixelIndex = h * width + w;

        // Calculate the starting value for the pixel in the input image
        const int inputIndex = pixelIndex * 3; // Times 3 as there are 3 input channels r, g, b

        // Access the RGB values
        const float red = inputImg[inputIndex];
        const float green = inputImg[inputIndex + 1];
        const float blue = inputImg[inputIndex + 2];

        // convert to greyscale
        const float grey = rFactor * red + gFactor * green + bFactor * blue;

        // Store the output
        outputImg[pixelIndex] = grey;
    }
}

imageToGreyConverter::imageToGreyConverter(float rFactor, float gFactor, float bFactor, const int width, const int height) {
    this->rFactor = rFactor;
    this->gFactor = gFactor;
    this->bFactor = bFactor;

    this->width = width;
    this->height = height;

    // Allocate GPU memory
    cudaMalloc(&this->outputImg, sizeof(float) * this->height * this->width * 1); // Greyscale
}

imageToGreyConverter::~imageToGreyConverter() {
    // Free GPU memory
    cudaFree(this->outputImg);
}

float *imageToGreyConverter::imageToGrey(const float *inputImg) const {
    // Assuming that the input is already allocated on the GPU

    // Launch a kernel to convert the image to grey
    dim3 blockDim(16, 16); // 16x16 threads per block (256 which is dividable by 32 as warps run in groups of 32)
    // Calculation of the grid dimensions below ensures that we always have enough blocks to cover the whole image
    dim3 gridDim((width + blockDim.x - 1) / blockDim.x, (height + blockDim.y - 1) / blockDim.y);

    // Launch the kernel
    toGreyKernel<<<gridDim, blockDim>>>(inputImg, outputImg, width, height, rFactor, gFactor, bFactor);
    cudaDeviceSynchronize(); // Wait for conversion to finish

    // Return pointer to output on the GPU
    return outputImg;
}
