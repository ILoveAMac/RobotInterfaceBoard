#include "visualServoing.h"

visualServoing::visualServoing(float imageHeight, float imageWidth) : pidController(KP_POOP, KI_POOP, KD_POOP, MIN_POOP, MAX_POOP, true)
{
    this->imageHeight = imageHeight;
    this->imageWidth = imageWidth;
}

visualServoing::~visualServoing() {}

std::vector<float> visualServoing::calculateControlPosition(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition)
{
    // Extract bounding box center
    float x_b = boundingBox[0]; // x is the center of the box

    // Calculate the error in the x direction
    float delta_x = x_b - CX; // Error in pixels from the image center

    // float rotation_speed = -0.0025 * delta_x;
    float rotation_speed = this->pidController.compute(delta_x, 0);

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