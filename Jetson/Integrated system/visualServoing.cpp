#include "visualServoing.h"

visualServoing::visualServoing(float imageHeight, float imageWidth) : pidController(KP_POOP, KI_POOP, KD_POOP, MIN_POOP, MAX_POOP, true),
                                                                      linearController(KP_LINEAR, KI_LINEAR, KD_LINEAR, MIN_LINEAR, MAX_LINEAR, true)
{
    this->imageHeight = imageHeight;
    this->imageWidth = imageWidth;

    this->currentState = servoingState::ROTATE;
}

visualServoing::~visualServoing() {}

std::vector<float> visualServoing::calculateControlPosition(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition, float dist5)
{
    // State machine for visual servoing
    switch (this->currentState)
    {
    case servoingState::ROTATE:
        return this->rotateState(boundingBox, robotCurrentPosition);
        break;
    case servoingState::MOVE_FORWARD:
        return this->moveForwardState(boundingBox, robotCurrentPosition, dist5);
        break;
    case servoingState::STOP:
        return robotCurrentPosition;
        break;
    default:
        return robotCurrentPosition;
        break;
    }
}

std::vector<float> visualServoing::rotateState(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition)
{
    // Extract bounding box center
    float x_b = boundingBox[0]; // x is the center of the box

    // Calculate the error in the x direction
    float delta_x = x_b - (CX - 10); // Error in pixels from the image center

    // float rotation_speed = -0.0025 * delta_x;
    float rotation_speed = this->pidController.compute(delta_x, 0);
    std::cout << "Rotation speed: " << rotation_speed << std::endl;

    // check if the rotation speed is sufficently low and transition to the vertical allignment state
    if (std::fabs(rotation_speed) < 0.048)
    {
        this->currentState = servoingState::MOVE_FORWARD; // State transition
        return {robotCurrentPosition[0], robotCurrentPosition[1], robotCurrentPosition[2]};
    }

    // Update only the robot's theta (rotation), keep x and y the same
    return {robotCurrentPosition[0], robotCurrentPosition[1], robotCurrentPosition[2] + rotation_speed};
}

std::vector<float> visualServoing::moveForwardState(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition, float dist5)
{
    // Extract bounding box center
    float y_b = boundingBox[1]; // y is the vertical center of the box

    // Calculate the error in the y direction
    float delta_y = y_b - (CY + 10); // Error in pixels from the image center vertically
    std::cout << "dy " << delta_y << std::endl;
    // Calculate the forward/backward speed based on delta_y
    // Here we will use the pixel difference for now, but eventually, you'll switch to a distance-based control
    float forward_speed = this->linearController.compute(delta_y, 0);

    // Extract the robot's current heading (theta) from robotCurrentPosition
    float theta = robotCurrentPosition[2]; // Assume theta is the third element (in radians)

    // Check if the error in y direction is small enough to stop
    if (std::fabs(delta_y) < 50) // Assuming a 10-pixel threshold for being "centered"
    {
        // If the robot is centered, stop moving and transition to the next state or stop
        this->currentState = servoingState::STOP; // Assuming you have a STOP state
        return {robotCurrentPosition[0], robotCurrentPosition[1], robotCurrentPosition[2]};
    }

    // Update the robot's position in the world frame
    // Forward movement affects both x and y positions based on the robot's heading (theta)
    float new_x = robotCurrentPosition[0] + forward_speed * std::cos(theta);
    float new_y = robotCurrentPosition[1] + forward_speed * std::sin(theta);

    std::cout << "dist: " << dist5 << std::endl;

    // std::cout << "new_x: " << new_x << " new_y: " << new_y << std::endl;
    // std::cout << "old_x: " << robotCurrentPosition[0] << " old_y: " << robotCurrentPosition[1] << std::endl;

    // Return the updated position with the new x, y, and unchanged theta
    return {new_x, new_y, theta};
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