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
#include <vector>
#include <cmath>

    class visualServoing
    {
    public:
        std::vector<float> calculateControlPosition(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition, std::vector<std::vector<float>> K);
    };

    std::vector<float> visualServoing::calculateControlPosition(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition, std::vector<std::vector<float>> K)
    {
        // Extract bounding box center
        float x_b = boundingBox[0]; // x is already the center of the box
        float y_b = boundingBox[1]; // y is already the center of the box

        // Camera intrinsic matrix
        float f_x = K[0][0];
        float c_x = K[0][2];
        float f_y = K[1][1];
        float c_y = K[1][2];

        // Normalize the bounding box center coordinates using camera intrinsics
        float u_prime = (x_b - c_x) / f_x;
        float v_prime = (y_b - c_y) / f_y;

        // Compute the target angle in the camera's 2D plane
        float target_angle = std::atan2(v_prime, u_prime); // angle relative to camera

        // Get the robot's current orientation (theta)
        float robot_theta = robotCurrentPosition[2]; // assuming theta is in radians

        // Compute the angular difference
        float delta_theta = target_angle - robot_theta;

        // Normalize the angle to the range [-pi, pi]
        if (delta_theta > M_PI)
        {
            delta_theta -= 2 * M_PI;
        }
        else if (delta_theta < -M_PI)
        {
            delta_theta += 2 * M_PI;
        }

        // Return the control position (delta_theta) for rotation
        return {robotCurrentPosition[0], robotCurrentPosition[1], delta_theta};
    }

    // // Step 1. Compute the relative 3D position of the bounding box center with respect to the camera
    // std::vector<float> relativePosition = computeRelativePosition(boundingBox);

    // // Step 2. Convert the camera coordinates to world coordinates
    // std::vector<float> objectPositionWithRespectToRobot = cameraToWorld(relativePosition);

    // // Step 3. Compute the desired robot position
    // // Get the current robot position
    // const float robotX = robotCurrentPosition[0];
    // const float robotY = robotCurrentPosition[1];
    // const float robotTheta = robotCurrentPosition[2];

    // // Get the desired robot position
    // const float dx = objectPositionWithRespectToRobot[0];
    // const float dy = objectPositionWithRespectToRobot[1];

    // // Calculate the distance and the angle to the object
    // const float distanceToTarget = sqrt(dx * dx + dy * dy);
    // const float desiredAngle = atan2(dx + robotX, dy + robotY);

    // // Print dx and dy and theta
    // std::cout << "dx :" << dx << " dy: " << dy << " theta: " << desiredAngle << std::endl;

    // // Calculate the desired robot position
    // float adjustedDistance = distanceToTarget - this->targetDist;
    // const float desiredX = robotX + adjustedDistance * cos(robotTheta + desiredAngle);
    // const float desiredY = robotY + adjustedDistance * sin(robotTheta + desiredAngle);

    // return {robotX, robotY, desiredAngle};
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
