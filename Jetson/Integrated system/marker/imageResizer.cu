#include "imageResizer.cuh"

__global__ void resizeKernel(const float *inputImg, float *outputImg, const int inW, const int inH, const int outW, const int outH, const int channels) {
    const int h = blockIdx.y * blockDim.y + threadIdx.y;
    const int w = blockIdx.x * blockDim.x + threadIdx.x;

    // Check that we are within the bounds of the output image
    if (w < outW && h < outH) {
        // Calculate the corresponding position in the input image
        int nearestX = static_cast<int>(roundf(static_cast<float>(w) * inW / outW));
        int nearestY = static_cast<int>(roundf(static_cast<float>(h) * inH / outH));

        // Ensure the indices are within the valid range
        nearestX = min(nearestX, inW - 1);
        nearestY = min(nearestY, inH - 1);

        // Map all the channels from input to output
        for (int c = 0; c < channels; ++c) {
            outputImg[(h * outW + w) * channels + c] = inputImg[(nearestY * inW + nearestX) * channels + c];
        }
    }
}

imageResizer::imageResizer(const int w, const int h) {
    this->w = w;
    this->h = h;

    // Allocate gpu memory for the output image
    cudaMalloc(&this->outputImg, sizeof(float) * this->h * this->w * 3); // RGB
}

imageResizer::~imageResizer() {
    // Free up gpu memory
    cudaFree(this->outputImg);
}

float* imageResizer::Resize(const int inputW, const int inputH, const float *data, const int channels) const {
    // Launch the resize kernel
    dim3 blockSize(16, 16);
    dim3 gridSize((this->w + blockSize.x - 1) / blockSize.x, (this->h + blockSize.y - 1) / blockSize.y);
    resizeKernel<<<gridSize, blockSize>>>(data, this->outputImg, inputW, inputH, this->w, this->h, channels);

    // Synchronize to ensure the kernel has finished
    cudaDeviceSynchronize();

    // return pointer to gpu memory
    return this->outputImg;
}



