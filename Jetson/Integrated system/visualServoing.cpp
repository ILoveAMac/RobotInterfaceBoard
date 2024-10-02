#include "visualServoing.h"

visualServoing::visualServoing(float imageHeight, float imageWidth)
{
    this->imageHeight = imageHeight;
    this->imageWidth = imageWidth;

    // Rotation matrix of the camera with respect to the ground
    // Uinity matrix for now //TODO!
    this->R = cv::Mat::eye(3, 3, CV_32F);

    this->targetDist = 0.2f;
}

visualServoing::~visualServoing() {}

void visualServoing::setTargetDist(float targetDist)
{
    this->targetDist = targetDist;
}

std::vector<float> visualServoing::calculateControlPosition(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition)
{
    // Extract bounding box center
    float x_b = boundingBox[0]; // x is the center of the box

    // Calculate the error in the x direction
    float delta_x = x_b - CX; // Error in pixels from the image center

    // Proportional control: the larger the error, the more we rotate
    float rotation_speed = 0.001 * delta_x;

    std::cout << "Rotation speed: " << rotation_speed << std::endl;

    // Update only the robot's theta (rotation), keep x and y the same
    return {robotCurrentPosition[0], robotCurrentPosition[1], robotCurrentPosition[2] + rotation_speed};
}

std::vector<float> visualServoing::removeDistortion(std::vector<float> point)
{
    std::vector<float> undistortedPoint;

    // Step 1: Normalize the distorted pixel coordinates
    const double x_d = (point[0] - CX) / FX;
    const double y_d = (point[1] - CY) / FY;

    // Step 2: Initiate the undistorted pixel coordinates
    double x_u = x_d;
    double y_u = y_d;

    // Step 3: Iteratively remove distortion
    for (int iter = 0; iter < MAX_ITER_DIST; iter++)
    {
        const double x_u_prev = x_u;
        const double y_u_prev = y_u;

        const double r_u2 = x_u * x_u + y_u * y_u;
        const double D = 1 + K1_DIST * r_u2 + K2_DIST * r_u2 * r_u2;

        // Corrected update using x_d and y_d
        x_u = x_d / D;
        y_u = y_d / D;

        const double dx = x_u - x_u_prev;
        const double dy = y_u - y_u_prev;

        if ((dx * dx + dy * dy) < CONV_TOLERANCE_DIST * CONV_TOLERANCE_DIST)
        {
            break; // Converged
        }
    }
    // return and cast to float
    return {static_cast<float>(x_u), static_cast<float>(y_u)};
}

std::vector<float> visualServoing::computeRelativePosition(std::vector<float> boundingBox)
{
    // Remove the distortion from the bounding box center
    std::vector<float> undistortedCenter = removeDistortion({boundingBox[0], boundingBox[1]});
    // Points are now normalized

    // Assume an arbitrary Z = 1 unit distance for scaling purposes (relative depth)
    constexpr float Z = 0.2f; // TODO! Change and see how it affects the output
                              // TODO! The Z parameter is currently always 1, I will need some kind of depth estimation
                              // TODO! I can get this maybe using DIST sens 5, or maybe using the bounding box size

    // Calculate relative 3D coordinates in the camera frame
    float X = undistortedCenter[0] * Z;
    float Y = undistortedCenter[1] * Z;

    return {X, Y, Z}; // Return relative position in the camera frame
}

std::vector<float> visualServoing::cameraToWorld(std::vector<float> cameraCoordinates)
{
    // Applying rotation matrix R to convert camera frame coordinates to world frame
    float X_world = R.at<float>(0, 0) * cameraCoordinates[0] + R.at<float>(0, 1) * cameraCoordinates[1] + R.at<float>(0, 2) * cameraCoordinates[2];
    float Y_world = R.at<float>(1, 0) * cameraCoordinates[0] + R.at<float>(1, 1) * cameraCoordinates[1] + R.at<float>(1, 2) * cameraCoordinates[2];
    float Z_world = R.at<float>(2, 0) * cameraCoordinates[0] + R.at<float>(2, 1) * cameraCoordinates[1] + R.at<float>(2, 2) * cameraCoordinates[2];

    return {X_world, Y_world, Z_world};
}
