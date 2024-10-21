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
#define FX_VS 330.582565765307
#define FY_VS 587.847666790967
#define CX_VS 223.296633717336
#define CY_VS 235.301280343201

// Distortion coefficients
#define K1_DIST_VS 0.033724646482670
#define K2_DIST_VS -0.117593449553171

// Controller parameters
#define KP_POOP -0.0025
#define KI_POOP -0.001
#define KD_POOP 0
#define MIN_POOP -0.15
#define MAX_POOP 0.15

// Controller parameters for linear motion
#define KP_LINEAR -0.003
#define KI_LINEAR -0.02
#define KD_LINEAR 0
#define MIN_LINEAR -0.25
#define MAX_LINEAR 0.25

// Controller parameters for marker angular allignment
#define KP_MARKER_ROTATE 0.2
#define KI_MARKER_ROTATE 0.01
#define KD_MARKER_ROTATE 0
#define MIN_MARKER_ROTATE -0.15
#define MAX_MARKER_ROTATE 0.15

// Controller parameters for marker linear allignment
#define KP_MARKER_LINEAR 0.3
#define KI_MARKER_LINEAR 0.02
#define KD_MARKER_LINEAR 0
#define MIN_MARKER_LINEAR -0.25
#define MAX_MARKER_LINEAR 0.25

#include "pid.h"
#include "positionController.h"

enum class servoingState
{
    ROTATE,
    MOVE_FORWARD,
    STOP
};

class visualServoing
{
public:
    visualServoing(float imageHeight, float imageWidth);
    ~visualServoing();

    // Returns the updated robot position in [x, y, theta]
    // Return values are in meters and radians
    std::vector<float> calculateControlPosition(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition, positionController &controller);
    std::vector<float> calculateControlPositionMarker(std::tuple<std::vector<double>, std::vector<double>> markerVectors, std::vector<float> robotCurrentPosition, positionController &controller);

    // Function to get the current state machine state
    servoingState getCurrentState() { return this->currentState; }

    // Function to reset the state machine
    void resetState() { this->currentState = servoingState::ROTATE; }

private:
    // Function for angular allignment
    std::vector<float> rotateState(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition, positionController &controller);

    // function for linear allignment
    std::vector<float> moveForwardState(std::vector<float> boundingBox, std::vector<float> robotCurrentPosition, positionController &controller);

    // function for marker angular allignment
    std::vector<float> markerRotateState(std::tuple<std::vector<double>, std::vector<double>> markerVectors, std::vector<float> robotCurrentPosition);

    // function for marker linear allignment
    std::vector<float> markerMoveForwardState(std::tuple<std::vector<double>, std::vector<double>> markerVectors, std::vector<float> robotCurrentPosition, positionController &controller);

    float imageHeight;
    float imageWidth;

    // Controller for controling the allignment of the robot with the target
    PID pidController;

    PID linearController;

    // Marker allignment controller
    PID markerRotateController;

    PID markerLinearController;

    // current servoing state
    servoingState currentState;
};

#endif // VISUAL_SERVOING_H