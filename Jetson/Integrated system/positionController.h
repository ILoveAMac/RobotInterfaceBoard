#ifndef positionController_H
#define positionController_H

#include <vector>
#include <math.h>
#include <iostream>

#include "pid.h"

#define AXLE_LENGTH 0.307f
#define MAX_LINEAR_VELOCITY 0.3f
#define MAX_ANGULAR_VELOCITY 1.0f

// Angular Velocity PID controller parameters
#define A_KP 0.8f
#define A_KI 0.3f
#define A_KD 0.0f

#define ALPHA 0.8 // Smoothing factor

// State machine states
enum class State
{
    IDLE,
    ROTATE_TO_GOAL,
    MOVE_TO_GOAL,
    ROTATE_TO_GOAL_ORIENTATION
};

class positionController
{
public:
    positionController(float Kp, float Ka, float goalTolerance, float thetaTolerance);
    ~positionController();

    // Calculate the velocities to reach the goal position
    std::vector<float> updateVelocities(float x, float y, float theta);

    // Set the goal position
    void setGoal(float x, float y, float theta);

    // Set the goal tolerance
    void setGoalTolerance(float tolerance);

    // Set theta tolerance
    void setThetaTolerance(float tolerance);

    // Set the control gains
    void setGains(float Kp, float Ka);

private:
    // Internal functions
    float calculateAlpha(float x, float y, float theta);
    float calculateP(float x, float y);

    float calculateV(float alpha, float p);
    float calculateW(float alpha);

    std::vector<float> calculateVelocities(float V, float W);

    float normalizeAngle(float angle);

    // Functions for the state machine
    std::vector<float> rotateToGoal(float x, float y, float theta);
    std::vector<float> moveToGoal(float x, float y, float theta);
    std::vector<float> rotateToGoalOrientation(float theta);

    float goalTolerance;
    float thetaTolerance;
    float Kp;
    float Ka;

    // Goal position
    float goalX;
    float goalY;
    float goalTheta;

    // Variables to smooth the velocities
    float V_prev;
    float W_prev;

    // Current state
    State state;

    // PID controller for the angular velocity
    PID pidTheta;
};

#endif // positionController_H