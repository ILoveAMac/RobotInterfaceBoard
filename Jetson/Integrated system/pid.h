#ifndef PID_H
#define PID_H

#include <chrono>

#define INTEGRAL_MAX 1.0f
#define INTEGRAL_MIN -1.0f

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

class PID
{
public:
    PID(float kp, float ki, float kd, float min, float max, bool useSystemTime = false);
    ~PID();

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setDt(float dt);

    float compute(float error, float input);

    void reset();

private:
    float kp, ki, kd;
    float min, max;
    float dt;

    float integral;
    float prevError;

    float prevMeasurement;

    bool useSystemTime;
    double prevTime;
};

#endif // PID_H