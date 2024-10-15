//
// Created by Wihan on 2024-09-24.
//

#ifndef BLURIMAGE_CUH
#define BLURIMAGE_CUH

#include "applyKernelToImage.cuh"

class blurImage {
public:
    blurImage(int width, int height, int kernelSize, float sigma);

    ~blurImage();

    float* blur(const float* image, int width, int height) const;

private:
    static float* computeGaussianKernel(float sigma, int kernelSize);
    int inputWidth, inputHeight, kernelSize;
    float* d_kernel; // Pointer to the kernel stored on the GPU
    applyKernelToImage* kernelApplier; // Pointer to applyKernelToImage instance
};


#endif //BLURIMAGE_CUH
