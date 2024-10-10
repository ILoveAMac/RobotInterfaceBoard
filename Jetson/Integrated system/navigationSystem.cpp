#include "navigationSystem.h"

navigationSystem::navigationSystem()
{
    // Set navigation state to forward
    navigationState = NavigationState::FORWARD;

    // Set turn direction to left
    turnDirection = TurnDirection::LEFT;

    distanceSinceLastObstacle = 0.0;
}

navigationSystem::~navigationSystem()
{
}

std::vector<float> navigationSystem::explore(std::vector<float> robotPosition, std::vector<float> distMeasurements)
{
    switch (navigationState)
    {
    case NavigationState::FORWARD:
        return forwardState(robotPosition, distMeasurements);
    case NavigationState::BACKWARD:
        return backwardState(robotPosition, distMeasurements);
    case NavigationState::STOP:
        return stopState(robotPosition, distMeasurements);
    case NavigationState::AVOID_OBSTACLE:
        return avoidObstacleState(robotPosition, distMeasurements);
    case NavigationState::MOVE_AWAY_FROM_OBSTACLE:
        return moveAwayFromObstacleState(robotPosition, distMeasurements);
    case NavigationState::ATTEMPT_TO_PASS_OBSTACLE:
        return attemptToPassObstacleState(robotPosition, distMeasurements);
    case NavigationState::CHECK_IF_CLEAR:
        return checkIfClearState(robotPosition, distMeasurements);
    default:
        std::cout << "Invalid Navigation State" << std::endl;
        return stopState(robotPosition, distMeasurements);
    }
}

std::vector<float> navigationSystem::forwardState(std::vector<float> robotPosition, std::vector<float> distMeasurements)
{
    std::cout << "Moving forward" << std::endl;

    // Check if forward motion is possible, if not then change state to avoid obstacle
    if (!isForwardMotionPossible(distMeasurements))
    {
        navigationState = NavigationState::AVOID_OBSTACLE;
        return avoidObstacleState(robotPosition, distMeasurements);
    }

    // Robot position is stored as x, y, theta
    // Move the robot forward by FORWRD_MOVE_DISTANCE in the direction of theta

    // Calculate the new x and y position
    std::vector<float> newPosition = robotPosition;
    newPosition[0] += FORWRD_MOVE_DISTANCE * cos(robotPosition[2]);
    newPosition[1] += FORWRD_MOVE_DISTANCE * sin(robotPosition[2]);

    // Return the new position
    return newPosition;
}

std::vector<float> navigationSystem::backwardState(std::vector<float> robotPosition, std::vector<float> distMeasurements)
{
    // TODO! Implement if needed
    return robotPosition;
}

std::vector<float> navigationSystem::stopState(std::vector<float> robotPosition, std::vector<float> distMeasurements)
{
    // Simply return the robot position
    return robotPosition;
}

std::vector<float> navigationSystem::avoidObstacleState(std::vector<float> robotPosition, std::vector<float> distMeasurements)
{
    std::cout << "Avoiding obstacle" << std::endl;

    // Robot position is stored as x, y, theta
    // Turn away from the obstace in the turn direction
    float newAngle = turnDirectionToAngle(this->turnDirection, robotPosition);

    // Alternate the turn direction
    alternateTurnDirection();

    // Get new robot position
    std::vector<float> newPosition = robotPosition;
    newPosition[2] = newAngle;

    // Change state to move away from obstacle
    navigationState = NavigationState::MOVE_AWAY_FROM_OBSTACLE;

    // Return the new position
    return newPosition;
}

std::vector<float> navigationSystem::moveAwayFromObstacleState(std::vector<float> robotPosition, std::vector<float> distMeasurements)
{
    std::cout << "Moving away from obstacle" << std::endl;

    // Check if forward motion is possible, if not go to attempt to pass obstacle state
    if (!isForwardMotionPossible(distMeasurements))
    {
        navigationState = NavigationState::ATTEMPT_TO_PASS_OBSTACLE;
        return attemptToPassObstacleState(robotPosition, distMeasurements);
    }

    // Move forwards in increments of 0.3 meters, transition to attempt to pass obstacle state if the distance since last obstacle is greater than 0.5 meters
    if (this->distanceSinceLastObstacle > 1.0)
    {
        this->distanceSinceLastObstacle = 0.0;

        navigationState = NavigationState::ATTEMPT_TO_PASS_OBSTACLE;
        return attemptToPassObstacleState(robotPosition, distMeasurements);
    }

    this->distanceSinceLastObstacle += 0.3;

    // Robot position is stored as x, y, theta
    // Move the robot forward by 0.3 meters in the direction of theta
    std::vector<float> newPosition = robotPosition;
    newPosition[0] += 0.3 * cos(robotPosition[2]);
    newPosition[1] += 0.3 * sin(robotPosition[2]);

    // Return the new position
    return newPosition;
}

std::vector<float> navigationSystem::attemptToPassObstacleState(std::vector<float> robotPosition, std::vector<float> distMeasurements)
{
    std::cout << "Attempting to pass obstacle" << std::endl;

    // Turn in the current turn direction
    float newAngle = turnDirectionToAngle(this->turnDirection, robotPosition);

    // Get new robot position
    std::vector<float> newPosition = robotPosition;
    newPosition[2] = newAngle;

    // Change state to move away from obstacle
    navigationState = NavigationState::CHECK_IF_CLEAR;

    // Return the new position
    return newPosition;
}

std::vector<float> navigationSystem::checkIfClearState(std::vector<float> robotPosition, std::vector<float> distMeasurements)
{
    std::cout << "Checking if clear" << std::endl;

    // If forward motion is not possible turn 180 degrees and move away from the obstacle
    if (!isForwardMotionPossible(distMeasurements))
    {
        float newAngle = robotPosition[2] + M_PI;

        // Normalize the angle
        while (newAngle > M_PI)
        {
            newAngle -= 2 * M_PI;
        }
        while (newAngle < -M_PI)
        {
            newAngle += 2 * M_PI;
        }

        // Get new robot position
        std::vector<float> newPosition = robotPosition;
        newPosition[2] = newAngle;

        // Change state to move forward
        navigationState = NavigationState::FORWARD;

        // Return the new position
        return newPosition;
    }

    // Else it is possible, go back to moving forward state

    // Also alternate the turn direction
    alternateTurnDirection();

    navigationState = NavigationState::FORWARD;
    return forwardState(robotPosition, distMeasurements);
}

NavigationState navigationSystem::getNavigationState()
{
    return this->navigationState;
}

void navigationSystem::setPositionController(positionController *posController)
{
    this->posController = posController;
}

bool navigationSystem::isForwardMotionPossible(std::vector<float> distMeasurements)
{
    bool forwardPossible = true;
    for (int i = 0; i < distMeasurements.size(); i++)
    {
        if (distMeasurements[i] < OBSTACLE_DETECTION_DISTANCE && distMeasurements[i] != -1)
        {
            forwardPossible = false;
            std::cout << "Obstacle detected at distance: " << distMeasurements[i] << std::endl;
            break;
        }
    }

    return forwardPossible;
}

float navigationSystem::turnDirectionToAngle(TurnDirection turnDirection, std::vector<float> robotPosition)
{
    // Convert the turn direction to angle
    // The angle is in radians
    // The angle is bounded between -pi and pi

    // We add the computed angle to the current theta of the robot

    float angle = 0.0;
    switch (turnDirection)
    {
    case TurnDirection::LEFT:
        angle = M_PI / 2;
        break;
    case TurnDirection::RIGHT:
        angle = -M_PI / 2;
        break;
    default:
        std::cout << "Invalid Turn Direction" << std::endl;
        break;
    }

    // get the new robot angle
    float newAngle = robotPosition[2] + angle;

    // Normalize the angle
    while (newAngle > M_PI)
    {
        newAngle -= 2 * M_PI;
    }
    while (newAngle < -M_PI)
    {
        newAngle += 2 * M_PI;
    }

    return newAngle;
}

void navigationSystem::alternateTurnDirection()
{
    switch (this->turnDirection)
    {
    case TurnDirection::LEFT:
        this->turnDirection = TurnDirection::RIGHT;
        break;
    case TurnDirection::RIGHT:
        this->turnDirection = TurnDirection::LEFT;
        break;
    default:
        std::cout << "Invalid Turn Direction" << std::endl;
        break;
    }
}
