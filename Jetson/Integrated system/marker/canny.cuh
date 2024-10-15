//
// Created by Wihan on 2024-09-24.
//

#ifndef CANNY_CUH
#define CANNY_CUH
#define PI 3.141592653589793238462643383279502884197

#include <stack>
#include <utility> // for std::pair
#include "blurImage.cuh"
#include "applySobel.cuh"



class canny {
public:
    canny(int width, int height, float sigma, int kernelSize, float minThreshold, float maxThreshold, int nmsLowerThreshold);
    ~canny();

    float* applyCanny(const float* input) const;
private:

    float* non_max_suppression(const float* mag, const float* theta) const;
    float* hysteresis(const float* input) const;

    int width, height;
    float minThreshold, maxThreshold;
    float nmsLowerThreshold;
    blurImage blur;
    applySobel applySobel;
};



#endif //CANNY_CUH
