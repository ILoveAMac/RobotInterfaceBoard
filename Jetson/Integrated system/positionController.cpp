#include "positionController.h"

positionController::positionController(float Kp, float Ka, float goalTolerance, float thetaTolerance) : pidTheta(A_KP, A_KI, A_KD, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY, true)
{
    this->Kp = Kp;
    this->Ka = Ka;
    this->goalTolerance = goalTolerance;
    this->thetaTolerance = thetaTolerance;

    this->goalX = 0;
    this->goalY = 0;
    this->goalTheta = 0;

    this->V_prev = 0;
    this->W_prev = 0;

    // Current state
    this->state = State::IDLE;
}

positionController::~positionController() {}

// update velocities from the current position
std::vector<float> positionController::updateVelocities(float x, float y, float theta)
{
    switch (state)
    {
    case State::IDLE:
        return {0, 0}; // Return zero velocities

    case State::ROTATE_TO_GOAL:
        return rotateToGoal(x, y, theta);

    case State::MOVE_TO_GOAL:
        return moveToGoal(x, y, theta);

    case State::ROTATE_TO_GOAL_ORIENTATION:
        return rotateToGoalOrientation(theta);

    default:
        // Handle unexpected states gracefully
        return {0, 0};
    }
}

std::vector<float> positionController::rotateToGoal(float x, float y, float theta)
{
    float alpha = calculateAlpha(x, y, theta);

    printf("alpha: %f\n", alpha);

    if (fabs(alpha) < thetaTolerance)
    {
        state = State::MOVE_TO_GOAL; // Transition to moving to the goal
        return {0, 0};               // Stop rotating
    }

    // PID control on the rotation
    float W = pidTheta.compute(alpha, theta);
    return calculateVelocities(0, W); // Only rotate
}

std::vector<float> positionController::moveToGoal(float x, float y, float theta)
{
    float distance = calculateP(x, y);

    if (distance < goalTolerance)
    {
        state = State::ROTATE_TO_GOAL_ORIENTATION; // Transition to rotating to goal orientation
        return {0, 0};
    }

    float alpha = calculateAlpha(x, y, theta);
    float V = calculateV(alpha, distance);
    float W = calculateW(alpha);

    return calculateVelocities(V, W);
}

std::vector<float> positionController::rotateToGoalOrientation(float theta)
{
    float angleError = normalizeAngle(goalTheta - theta);

    if (fabs(angleError) < thetaTolerance)
    {
        state = State::IDLE; // Goal reached, stop
        // Notify the caller that the goal has been reached
        std::cout << "Goal reached" << std::endl;
        return {0, 0};
    }

    // PID control for rotation to goal orientation
    float W = pidTheta.compute(angleError, theta);
    return calculateVelocities(0, W); // Only rotate
}

// Set the goal position
void positionController::setGoal(float x, float y, float theta)
{
    this->goalX = x;
    this->goalY = y;
    this->goalTheta = theta;

    // Reset the PID controllers to avoid jumps in the control signal
    pidTheta.reset();

    this->state = State::ROTATE_TO_GOAL;
}

// Set the goal tolerance
void positionController::setGoalTolerance(float tolerance)
{
    this->goalTolerance = tolerance;
}

// Set theta tolerance
void positionController::setThetaTolerance(float tolerance)
{
    this->thetaTolerance = tolerance;
}

// Set the control gains
void positionController::setGains(float Kp, float Ka)
{
    this->Kp = Kp;
    this->Ka = Ka;
}

// Private functions
float positionController::calculateAlpha(float x, float y, float theta)
{
    // calculate the angle to the goal x,y position
    float beta = atan2(this->goalY - y, this->goalX - x);

    // calculate the angle to the goal orientation and normalize it
    return normalizeAngle(beta - theta);
}

float positionController::calculateP(float x, float y)
{
    // calculate the distance to the goal
    return sqrt(pow(this->goalX - x, 2) + pow(this->goalY - y, 2));
}

float positionController::calculateV(float alpha, float p)
{
    // calculate the linear velocity
    float V = this->Kp * p * cos(alpha);

    // check if the linear velocity is greater than the maximum
    if (V > MAX_LINEAR_VELOCITY)
    {
        return MAX_LINEAR_VELOCITY;
    }

    if (V < -MAX_LINEAR_VELOCITY)
    {
        return -MAX_LINEAR_VELOCITY;
    }

    return V;
}

float positionController::calculateW(float alpha)
{
    // calculate the angular velocity
    float W = this->Kp * sin(alpha) * cos(alpha) + this->Ka * alpha;

    // check if the angular velocity is greater than the maximum
    if (W > MAX_ANGULAR_VELOCITY)
    {
        return MAX_ANGULAR_VELOCITY;
    }

    if (W < -MAX_ANGULAR_VELOCITY)
    {
        return -MAX_ANGULAR_VELOCITY;
    }

    return W;
}

std::vector<float> positionController::calculateVelocities(float V, float W)
{
    // will return velocities for left and right side so 2 elements in the vector

    // Smooth the velocities
    V = ALPHA * V + (1 - ALPHA) * V_prev;
    W = ALPHA * W + (1 - ALPHA) * W_prev;

    // Save the previous velocities
    V_prev = V;
    W_prev = W;

    // The robot is not at the goal, so calculate the velocities to reach the goal

    float left = V - W * AXLE_LENGTH / 2;
    float right = V + W * AXLE_LENGTH / 2;

    if (fabs(left) < 0.05)
    {
        left = 0;
    }

    if (fabs(right) < 0.05)
    {
        right = 0;
    }

    return {left, right};
}

float positionController::normalizeAngle(float angle)
{
    angle = fmod(angle + M_PI, 2.0f * M_PI);
    if (angle < 0)
        angle += 2.0f * M_PI;
    angle -= M_PI;

    // Handle edge case near pi
    if (fabs(angle - M_PI) < 1e-3f)
        angle = -M_PI;

    return angle;
}