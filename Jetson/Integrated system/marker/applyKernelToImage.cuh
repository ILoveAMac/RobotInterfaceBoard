//
// Created by Wihan on 2024-09-24.
//

#ifndef APPLYKERNELTOIMAGE_CUH
#define APPLYKERNELTOIMAGE_CUH


class applyKernelToImage {
public:
    applyKernelToImage(int width, int height, int kernelWidth, int kernelHeight, float *kernel);
    ~applyKernelToImage();

    float* apply(const float* inputImg) const;

private:
    int kernelWidth;
    int kernelHeight;
    float* kernel;

    int inputWidth;
    int inputHeight;

    // Scratch space for output kernel
    float *output;
};


#endif //APPLYKERNELTOIMAGE_CUH
