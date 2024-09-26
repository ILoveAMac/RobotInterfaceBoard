#include "positionController.h"

positionController::positionController(float Kp, float Ka, float goalTolerance, float thetaTolerance)
{
    this->Kp = Kp;
    this->Ka = Ka;
    this->goalTolerance = goalTolerance;
    this->thetaTolerance = thetaTolerance;
    this->atGoal = false;
    this->atGoalTheta = false;

    this->goalX = 0;
    this->goalY = 0;
    this->goalTheta = 0;

    this->V_prev = 0;
    this->W_prev = 0;
}

positionController::~positionController() {}

// update velocities from the current position
std::vector<float> positionController::updateVelocities(float x, float y, float theta)
{
    // Calculate the angle to the goal position
    float alpha = calculateAlpha(x, y, theta);

    // Calculate the distance to the goal position
    float p = calculateP(x, y);

    // Calculate the linear velocity
    float V = calculateV(alpha, p);

    // Calculate the angular velocity
    float W = calculateW(alpha);

    // Calculate the left and right wheel velocities
    return calculateVelocities(V, W);
}

// Set the goal position
void positionController::setGoal(float x, float y, float theta)
{
    this->goalX = x;
    this->goalY = y;
    this->goalTheta = theta;
    this->atGoal = false;
    this->atGoalTheta = false;
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
    // calculate the distance to the goal
    float distance = sqrt(pow(this->goalX - x, 2) + pow(this->goalY - y, 2));

    // Check if the robot is at the goal
    if (distance < this->goalTolerance)
    {
        // set the robot at the goal
        this->atGoal = true;
    }

    if (this->atGoal)
    {
        if (fabs(this->goalTheta - theta) < this->thetaTolerance)
        {
            // the robot is at the goal
            this->atGoalTheta = true;
            return 0;
        }

        // the robot is at the goal, so only rotate in place
        printf("Goal theta: %f, theta: %f\n", this->goalTheta, theta);
        printf("diff: %f\n", this->goalTheta - theta);
        return this->goalTheta - theta;
    }

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
    float W = 0;
    if (this->atGoal)
    {
        W = this->Ka * alpha;
    }
    else
    {
        W = this->Kp * sin(alpha) * cos(alpha) + this->Ka * alpha;
    }

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

    // Check if the robot is at the goal, if it is then only rotate in place
    if (this->atGoal)
    {
        float left = -1 * W * AXLE_LENGTH / 2;
        float right = W * AXLE_LENGTH / 2;

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
    // Normalize the angle to be between -pi and pi
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;

    // Treat angles close to pi and -pi as the same to avoid oscillation
    if (fabs(angle - M_PI) < 1e-3)
        angle = -M_PI;

    return angle;
}