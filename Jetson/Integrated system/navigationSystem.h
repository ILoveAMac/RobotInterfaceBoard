#ifndef NAVIGATION_SYSTEM_H
#define NAVIGATION_SYSTEM_H

#include <vector>
#include <cmath>
#include <iostream>

#include "positionController.h"

// in meters
#define FORWRD_MOVE_DISTANCE 1.0

#define OBSTACLE_DETECTION_DISTANCE 0.3
#define SIDE_SENSOR_DETECTION_DISTANCE 0.2

#define OBSTACLE_AVOIDANCE_DISTANCE 1.0
#define OBSTACLE_AVOIDANCE_INCREMENT_DISTANCE 0.5

#define SIDE_SENSOR_CORRECTION_DISTANCE 0.3

// in radians, 10 degrees
#define SIDE_SENSOR_CORRECTION_TURN_ANGLE 0.34

enum class NavigationState
{
    FORWARD,
    BACKWARD,
    STOP,
    AVOID_OBSTACLE,
    MOVE_AWAY_FROM_OBSTACLE,
    ATTEMPT_TO_PASS_OBSTACLE,
    CHECK_IF_CLEAR,
    SIDE_SENSOR_CORRECTION_LEFT,
    SIDE_SENSOR_CORRECTION_RIGHT
};

enum class TurnDirection
{
    LEFT,
    RIGHT,
};

class navigationSystem
{
public:
    navigationSystem();
    ~navigationSystem();

    std::vector<float> explore(std::vector<float> robotPosition, std::vector<float> distMeasurements);

    // Function to get the navigation state
    NavigationState getNavigationState();

    // Function to set the position controller
    void setPositionController(positionController *posController);

private:
    // Functions for the state machine
    std::vector<float> forwardState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
    std::vector<float> backwardState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
    std::vector<float> stopState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
    std::vector<float> avoidObstacleState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
    std::vector<float> moveAwayFromObstacleState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
    std::vector<float> attemptToPassObstacleState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
    std::vector<float> checkIfClearState(std::vector<float> robotPosition, std::vector<float> distMeasurements);

    std::vector<float> sideSensorCorrectionLeftState(std::vector<float> robotPosition, std::vector<float> distMeasurements);
    std::vector<float> sideSensorCorrectionRightState(std::vector<float> robotPosition, std::vector<float> distMeasurements);

    // Function to check if forward motion is possible using distance sensor readings
    bool isForwardMotionPossible(std::vector<float> distMeasurements);

    // Function to check if any sensor reading is less than the obstacle detection distance, except one of the side sensor
    bool isObstacleDetected(std::vector<float> distMeasurements, int sensorToIgnore);

    // Function to check if we should transistion to one of the side sensor correction states
    bool shouldTransitionToSideSensorCorrection(std::vector<float> distMeasurements);

    NavigationState getSideSensorCorrectionState(std::vector<float> distMeasurements);

    // Function to convert turn direction to real world angle
    float turnDirectionToAngle(TurnDirection turnDirection, std::vector<float> robotPosition);

    void alternateTurnDirection();

    // Navigation state variable
    NavigationState navigationState;
    NavigationState previousNavigationState; // Used in the side sensor correction states

    // Turn direction variable
    // Thist variable indicates the next turn direction after the robot has detected an obstacle
    // It cycles between LEFT and RIGHT after each detection and avoidance of an obstacle
    TurnDirection turnDirection;

    // Distance traveled since the last obstacle was detected
    // When this exceeds some threshold, the robot turns back
    float distanceSinceLastObstacle;

    // Variable to store ammount turned in the current side sensor correction state
    float sideSensorCorrectionTurned;

    // Position controller object
    positionController *posController;
};
#endif // NAVIGATION_SYSTEM_H