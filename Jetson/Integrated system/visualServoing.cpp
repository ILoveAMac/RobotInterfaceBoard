#include "visualServoing.h"

visualServoing::visualServoing(float imageHeight, float imageWidth) : pidController(KP_POOP, KI_POOP, KD_POOP, MIN_POOP, MAX_POOP, true),
                                                                      linearController(KP_LINEAR, KI_LINEAR, KD_LINEAR, MIN_LINEAR, MAX_LINEAR, true),
                                                                      markerRotateController(KP_MARKER_ROTATE, KI_MARKER_ROTATE, KD_MARKER_ROTATE, MIN_MARKER_ROTATE, MAX_MARKER_ROTATE, true),
                                                                      markerLinearController(KP_MARKER_LINEAR, KI_MARKER_LINEAR, KD_MARKER_LINEAR, MIN_MARKER_LINEAR, MAX_MARKER_LINEAR, true)
{
    this->imageHeight = imageHeight;
    this->imageWidth = imageWidth;

    this->currentState = servoingState::ROTATE;
}

visualServoing::~visualServoing() {}

std::vector<float> visualServoing::calculateControlPosition(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition, positionController &controller)
{
    // State machine for visual servoing
    switch (this->currentState)
    {
    case servoingState::ROTATE:
        return this->rotateState(boundingBox, robotCurrentPosition, controller);
        break;
    case servoingState::MOVE_FORWARD:
        return this->moveForwardState(boundingBox, robotCurrentPosition, controller);
        break;
    case servoingState::STOP:
        return robotCurrentPosition;
        break;
    default:
        return robotCurrentPosition;
        break;
    }
}

std::vector<float> visualServoing::calculateControlPositionMarker(std::tuple<std::vector<double>, std::vector<double>> markerVectors, std::vector<float> robotCurrentPosition, positionController &controller)
{
    switch (this->currentState)
    {
    case servoingState::ROTATE:
        return this->markerRotateState(markerVectors, robotCurrentPosition);
        break;
    case servoingState::MOVE_FORWARD:
        return this->markerMoveForwardState(markerVectors, robotCurrentPosition, controller);
        break;
    case servoingState::STOP:
        return robotCurrentPosition;
        break;
    default:
        return robotCurrentPosition;
        break;
    }
}

std::vector<float> visualServoing::rotateState(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition, positionController &controller)
{
    controller.setReverseMode(false);

    // ========== FORWARD CHECK ==========
    // Check if we are tool close, if so move back
    // Too close is when the error for forward movement is negative
    float y_b = boundingBox[1];   // y is the vertical center of the box
    y_b += boundingBox[2] / 2.0f; // We add half the width of the bounding box to the center to get the front of the box

    // Calculate the error in the y direction
    float delta_y = y_b - (CY_VS + 100); // Error in pixels from the image center vertically

    // Calculate the forward/backward speed based on delta_y
    // Here we will use the pixel difference for now, but eventually, you'll switch to a distance-based control
    float forward_speed = this->linearController.compute(delta_y, 0);

    if (forward_speed < 0 && std::fabs(forward_speed) > 0.2)
    {
        controller.setReverseMode(true);
        float theta = robotCurrentPosition[2]; // Assume theta is the third element (in radians)
        float new_x = robotCurrentPosition[0] + forward_speed * std::cos(theta);
        float new_y = robotCurrentPosition[1] + forward_speed * std::sin(theta);

        // Return the updated position with the new x, y, and unchanged theta
        return {new_x, new_y, theta};
    }

    // ========== ROTATION CHECK ==========
    // Extract bounding box center
    float x_b = boundingBox[0]; // x is the center of the box

    // Calculate the error in the x direction
    float delta_x = x_b - (CX_VS + 20); // Error in pixels from the image center

    // float rotation_speed = -0.0025 * delta_x;
    float rotation_speed = this->pidController.compute(delta_x, 0);
    std::cout << "Rotation speed: " << rotation_speed << std::endl;

    // check if the rotation speed is sufficently low and transition to the vertical allignment state
    if (std::fabs(rotation_speed) < 0.05)
    {
        this->currentState = servoingState::MOVE_FORWARD; // State transition
        return {robotCurrentPosition[0], robotCurrentPosition[1], robotCurrentPosition[2]};
    }

    // Update only the robot's theta (rotation), keep x and y the same
    return {robotCurrentPosition[0], robotCurrentPosition[1], robotCurrentPosition[2] + rotation_speed};
}

std::vector<float> visualServoing::moveForwardState(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition, positionController &controller)
{
    std::cout << "Moving forward" << std::endl;

    // ========== ROTATION CHECK ==========
    // disable reverse mode
    controller.setReverseMode(false);

    // Extract bounding box center
    float x_b = boundingBox[0]; // x is the center of the box

    // Calculate the error in the x direction
    float delta_x = x_b - (CX_VS + 20); // Error in pixels from the image center

    // float rotation_speed = -0.0025 * delta_x;
    float rotation_speed = this->pidController.compute(delta_x, 0);
    std::cout << "Rotation speed: " << rotation_speed << std::endl;

    // check if the rotation speed is sufficently low and transition to the vertical allignment state
    if (std::fabs(rotation_speed) > 0.06)
    {
        this->currentState = servoingState::ROTATE; // State transition
        return {robotCurrentPosition[0], robotCurrentPosition[1], robotCurrentPosition[2]};
    }

    // ========== FORWARD CHECK ==========
    // Extract bounding box center
    float y_b = boundingBox[1];   // y is the vertical center of the box
    y_b += boundingBox[2] / 2.0f; // We add half the width of the bounding box to the center to get the front of the box

    // Calculate the error in the y direction
    float delta_y = y_b - (CY_VS + 100); // Error in pixels from the image center vertically

    // Calculate the forward/backward speed based on delta_y
    // Here we will use the pixel difference for now, but eventually, you'll switch to a distance-based control
    float forward_speed = this->linearController.compute(delta_y, 0);

    // Extract the robot's current heading (theta) from robotCurrentPosition
    float theta = robotCurrentPosition[2]; // Assume theta is the third element (in radians)

    // Check if the error in y direction is small enough to stop
    if (std::fabs(forward_speed) < 0.2) // Assuming a 10-pixel threshold for being "centered"
    {
        std::cout << "stop" << std::endl;
        // If the robot is centered, stop moving and transition to the next state or stop
        this->currentState = servoingState::STOP; // Assuming you have a STOP state
        return {robotCurrentPosition[0], robotCurrentPosition[1], robotCurrentPosition[2]};
    }

    if (forward_speed < 0)
    {
        controller.setReverseMode(true);
    }

    // Update the robot's position in the world frame
    // Forward movement affects both x and y positions based on the robot's heading (theta)
    float new_x = robotCurrentPosition[0] + forward_speed * std::cos(theta);
    float new_y = robotCurrentPosition[1] + forward_speed * std::sin(theta);

    std::cout << "Forward speed: " << forward_speed << std::endl;

    // Return the updated position with the new x, y, and unchanged theta
    return {new_x, new_y, theta};
}

std::vector<float> visualServoing::markerRotateState(std::tuple<std::vector<double>, std::vector<double>> markerVectors, std::vector<float> robotCurrentPosition)
{
    // We must rotate the robot to align with the marker
    // The second item in the tuple contains the marker's estimated roll, pitch, and yaw
    // We'll use the yaw to rotate the robot

    // Extract the yaw from the marker's estimated roll, pitch, and yaw
    float yaw = std::get<1>(markerVectors)[2];

    // Calculate the error in the yaw direction
    float delta_yaw = yaw - robotCurrentPosition[2]; // Error in radians from the robot's current heading
    // Print the error in yaw
    std::cout << "Error in yaw: " << delta_yaw << std::endl;

    // Check if the update is sufficiently low
    if (std::fabs(delta_yaw) < 0.05)
    {
        this->currentState = servoingState::MOVE_FORWARD; // State transition
        return {robotCurrentPosition[0], robotCurrentPosition[1], robotCurrentPosition[2]};
    }

    // Calculate the correction, we dont use a pid here, we use the error directly
    float corrected_yaw = robotCurrentPosition[2] + yaw * 1.0f;

    // Update only the robot's theta (rotation), keep x and y the same
    return {robotCurrentPosition[0], robotCurrentPosition[1], corrected_yaw};
}

std::vector<float> visualServoing::markerMoveForwardState(std::tuple<std::vector<double>, std::vector<double>> markerVectors, std::vector<float> robotCurrentPosition, positionController &controller)
{
    // just return the current position for now
    this->currentState = servoingState::STOP;
    return robotCurrentPosition;
}
