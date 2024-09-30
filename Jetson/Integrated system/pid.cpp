#include "pid.h"

PID::PID(float kp, float ki, float kd, float min, float max, bool useSystemTime)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    this->min = min;
    this->max = max;

    this->integral = 0.0f;
    this->prevError = 0.0f;
    this->prevMeasurement = 0.0f;

    this->useSystemTime = useSystemTime;

    if (useSystemTime)
    {
        this->prevTime = duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch()).count();
    }
    else
    {
        this->dt = 0.0f;
    }
}

PID::~PID()
{
}

void PID::setGains(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PID::setOutputLimits(float min, float max)
{
    this->min = min;
    this->max = max;
}

void PID::setDt(float dt)
{
    this->dt = dt;
}

float PID::compute(float error, float input)
{
    // Compute time difference or use dt if provided
    float dt_to_use;
    if (useSystemTime)
    {
        double currentTime = duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch()).count();
        dt_to_use = (currentTime - prevTime) / 1000.0f; // Convert to seconds
        prevTime = currentTime;
    }
    else
    {
        dt_to_use = dt;
    }

    // Handle the case where dt is zero (to avoid division by zero)
    if (dt_to_use <= 0.0f)
    {
        dt_to_use = 1e-6f; // A small positive value
    }

    // Integral term
    integral += error * dt_to_use;

    // Integral term with anti-windup
    integral += error * dt_to_use;
    if (integral > INTEGRAL_MAX)
    {
        integral = INTEGRAL_MAX;
    }
    else if (integral < INTEGRAL_MIN)
    {
        integral = INTEGRAL_MIN;
    }

    // Derivative term
    float derivative = (input - prevMeasurement) / dt_to_use;

    // Compute PID output
    float output = kp * error + ki * integral + kd * derivative;

    // Update variables for next iteration
    prevError = error;
    prevMeasurement = input;

    // Limit output
    if (output > max)
    {
        output = max;
    }
    else if (output < min)
    {
        output = min;
    }

    return output;
}

void PID::reset()
{
    // reset the parameters for the PID controller
    integral = 0.0f;
    prevError = 0.0f;
    prevMeasurement = 0.0f;
}
