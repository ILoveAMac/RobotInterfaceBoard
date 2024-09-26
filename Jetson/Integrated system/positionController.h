#ifndef positionController_H
#define positionController_H

#include <vector>
#include <math.h>

#include <iostream>

#define AXLE_LENGTH 0.307f
#define MAX_LINEAR_VELOCITY 0.5f
#define MAX_ANGULAR_VELOCITY 1.0f

#define ALPHA 0.9 // Smoothing factor

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

    // Check if the robot is at the goal
    bool atGoalPosition() { return atGoal; }

    // Check if the robot is at the goal orientation
    bool atGoalOrientation() { return atGoalTheta; }

private:
    // Internal functions
    float calculateAlpha(float x, float y, float theta);
    float calculateP(float x, float y);

    float calculateV(float alpha, float p);
    float calculateW(float alpha);

    std::vector<float> calculateVelocities(float V, float W);

    float normalizeAngle(float angle);

    float goalTolerance;
    float thetaTolerance;
    float Kp;
    float Ka;

    // Goal position
    float goalX;
    float goalY;
    float goalTheta;

    bool atGoal;
    bool atGoalTheta;

    // Variables to smooth the velocities
    float V_prev;
    float W_prev;
};

#endif // positionController_H