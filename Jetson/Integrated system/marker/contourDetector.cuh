//
// Created by Wihan on 2024-09-24.
//

#ifndef CONTOURDETECTOR_CUH
#define CONTOURDETECTOR_CUH
#include <vector>
#include <tuple>

#define DISTANCE_THRESHOLD 30

class contourDetector
{
public:
    contourDetector();
    ~contourDetector();

    static std::vector<std::vector<int>> detect(const float *image, int width, int height);

private:
    static bool isValid(int x, int y, int width, int height);
    static float calculateDistance(float x1, float y1, float x2, float y2);
};

#endif // CONTOURDETECTOR_CUH
