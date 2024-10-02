#ifndef VISUAL_SERVOING_H
#define VISUAL_SERVOING_H

#include <vector>
#include <cmath>

// In pixels
#define GOAL_THRESHOLD 10

// Camera Intrinsics and Distortion Coefficients
// K matrix
#define FX 441.824902112213
#define FY 588.842877481113
#define CX 222.518792028182
#define CY 234.806657834434

// Distortion coefficients
#define K1_DIST 0.0395769787967721
#define K2_DIST -0.161268896561332

// Maximum number of iterations to remove distortion
#define MAX_ITER_DIST 10
// Convergence tolerance for distortion removal
#define CONV_TOLERANCE_DIST 1e-6

// Controller parameters
#define KP_POOP -0.0025
#define KI_POOP -0.05
#define KD_POOP 0
#define MIN_POOP -0.3
#define MAX_POOP 0.3

// OpenCV Headers
#include <opencv2/opencv.hpp>

#include "pid.h"
class visualServoing
{
public:
    visualServoing(float imageHeight, float imageWidth);
    ~visualServoing();

    void setTargetDist(float targetDist);

    // Returns the updated robot position in [x, y, theta]
    // Return values are in meters and radians
    std::vector<float> calculateControlPosition(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition);

private:
    // Function to remove distortion from a point
    std::vector<float> removeDistortion(std::vector<float> point);

    // Function to compute the relative 3D position of the camera with respect to the target
    std::vector<float> computeRelativePosition(std::vector<float> boundingBox);

    // Function to convert the camera coordinates to world coordinates
    // These world coordinates are still however with respect to the camera, just in a different frame
    std::vector<float> cameraToWorld(std::vector<float> cameraCoordinates);

    float targetDist;

    float imageHeight;
    float imageWidth;

    // Rotation matrix of the camera with respect to the ground
    cv::Mat R;

    // Controller for controling the allignment of the robot with the target
    PID pidController;
};

#endif // VISUAL_SERVOING_H