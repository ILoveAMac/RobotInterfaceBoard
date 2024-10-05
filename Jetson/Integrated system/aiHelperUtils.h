#ifndef AIHELPERUTILS_H
#define AIHELPERUTILS_H

// Sorting algorithm for vectors
#include <bits/algorithmfwd.h>

// OpenCV for drawing bounding boxes
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

#define NUM_BOXES 2
#define GRID_SIZE 7
#define CONF_THRESH 0.4
#define IOU_NMS_THRESH 0.4
#define IMG_HEIGHT 448
#define IMG_WIDTH 448

class aiHelperUtils
{
public:
    aiHelperUtils();
    ~aiHelperUtils();

    // Bounding boxes
    static std::vector<std::vector<float>> getFinalBoundingBoxes(const float *detections);
    static cv::Mat drawBoundingBoxes(cv::Mat frame, std::vector<std::vector<float>> boxes);
    static void drawSensorReadingsOnFrame(cv::Mat &frame, const std::vector<float> &sensorData);
    static void drawSystemStateOnFrame(cv::Mat &frame, const std::string &stateString);
    static float getBoundingBoxArea(std::vector<float> box);
    static float getBoundingBoxConfidence(std::vector<float> box);
    static std::vector<float> getBoindingBoxWithLargestArea(std::vector<std::vector<float>> boxes);

private:
    static std::vector<std::vector<float>>
    nonMaxSuppression(std::vector<std::vector<float>> boxes);
    static float iou(std::vector<float> box1, std::vector<float> box2);
};

#endif // AIHELPERUTILS_H