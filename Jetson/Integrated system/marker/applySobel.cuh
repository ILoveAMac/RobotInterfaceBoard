//
// Created by Wihan on 2024-09-24.
//

#ifndef APPLYSOBEL_CUH
#define APPLYSOBEL_CUH

#include <vector>

#include "applyKernelToImage.cuh"

class applySobel {
public:
    applySobel(int width, int height);
    ~applySobel();

    std::pmr::vector<float*> apply(const float* inputImg) const;

private:
    float* arcTan2d(const float* G_x, const float* G_y) const;
    float* sqrt2D(const float* G_x, const float* G_y) const;

    float* sobel_x;
    float* sobel_y;
    applyKernelToImage* kernelApplierX;
    applyKernelToImage* kernelApplierY;

    int width;
    int height;
};



#endif //APPLYSOBEL_CUH
