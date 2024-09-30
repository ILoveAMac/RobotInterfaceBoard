#ifndef AIHELPERUTILS_H
#define AIHELPERUTILS_H

// Sorting algorithm for vectors
#include <bits/algorithmfwd.h>

#include <cuda_fp16.h>

// OpenCV for drawing bounding boxes
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

#define NUM_BOXES 2
#define GRID_SIZE 7
#define CONF_THRESH 0.5
#define IOU_NMS_THRESH 0.4
#define IMG_HEIGHT 448
#define IMG_WIDTH 448

class aiHelperUtils
{
public:
    aiHelperUtils();
    ~aiHelperUtils();

    // Bounding boxes
    static std::vector<std::vector<__half>> getFinalBoundingBoxes(const __half *detections);
    static cv::Mat drawBoundingBoxes(cv::Mat frame, std::vector<std::vector<__half>> boxes);

private:
    static std::vector<std::vector<__half>>
    nonMaxSuppression(std::vector<std::vector<__half>> boxes);
    static __half iou(std::vector<__half> box1, std::vector<__half> box2);
};

#endif // AIHELPERUTILS_H