#ifndef VISUAL_SERVOING_H
#define VISUAL_SERVOING_H

#include <vector>
#include <cmath>

// OpenCV Headers
#include <opencv2/opencv.hpp>

#include "aiHelperUtils.h"

// In pixels
#define GOAL_THRESHOLD 10

// Camera Intrinsics and Distortion Coefficients
// K matrix
#define FX 330.582565765307
#define FY 587.847666790967
#define CX 223.296633717336
#define CY 235.301280343201

// Distortion coefficients
#define K1_DIST 0.033724646482670
#define K2_DIST -0.117593449553171

// Maximum number of iterations to remove distortion
#define MAX_ITER_DIST 10
// Convergence tolerance for distortion removal
#define CONV_TOLERANCE_DIST 1e-6

// Controller parameters
#define KP_POOP -0.0025
#define KI_POOP -0.008
#define KD_POOP 0
#define MIN_POOP -0.15
#define MAX_POOP 0.15

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

    float imageHeight;
    float imageWidth;

    // Controller for controling the allignment of the robot with the target
    PID pidController;
};

#endif // VISUAL_SERVOING_H