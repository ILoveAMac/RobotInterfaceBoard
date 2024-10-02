#ifndef VISUAL_SERVOING_H
#define VISUAL_SERVOING_H

#include <vector>
#include <cmath>

// In pixels
#define GOAL_THRESHOLD 10

// Camera Intrinsics and Distortion Coefficients
// K matrix
#define FX 658.596215580897
#define FY 658.301628353110
#define CX 313.811167083958
#define CY 245.354670424866

// Distortion coefficients
#define K1 0.106606071597603
#define K2 -0.505400337387861

// Maximum number of iterations to remove distortion
#define MAX_ITER_DIST 10
// Convergence tolerance for distortion removal
#define CONV_TOLERANCE_DIST 1e-6

// OpenCV Headers
#include <opencv2/opencv.hpp>

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
};

#endif // VISUAL_SERVOING_H