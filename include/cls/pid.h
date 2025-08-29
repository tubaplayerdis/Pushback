//
// Created by aaron on 7/23/2025.
//

#ifndef PID_H
#define PID_H
#include <cmath>



//Value that is used to determine whether the power is limited or not. Max and Min are set to this by default to signify that no power limit should be in place
constexpr int MAX_MIN_NO_USE = 0;

template<typename T>
T clamp(T val, T mn, T mx){
    return std::max(std::min(val, mx), mn);
}

class pid
{
    const float KP;
    const float KI;
    const float KD;
    const float Tolerance;
    const float MaxOutput;
    const float MinOutput;

    //Operational, IE changing values
    float integral;
    float derivative;
    float lastError;

public:
    const float Target;

    virtual ~pid() = default;

    pid(float p, float i, float d, float target, float tolerance, float max = MAX_MIN_NO_USE, float min = MAX_MIN_NO_USE) : KP(p), KI(i), KD(d), Target(target), Tolerance(tolerance), MaxOutput(max), MinOutput(min), lastError(0), integral(0), derivative(0) {}

    //Call inside a loop. Returns needed power
    float Compute(float error);
};

inline float pid::Compute(float error)
{
    if (fabs(error) < Tolerance) return MinOutput;//Do not handle the PID loop

    integral += error;
    derivative = error - lastError;

    // Calculate power
    float power = (KP * error) + (KI * integral) + (KD * derivative);

    // Limit power
    if (MaxOutput != MAX_MIN_NO_USE || MinOutput != MAX_MIN_NO_USE)
    {
        power = clamp<float>(power, MinOutput, MaxOutput);
    }

    return power;//Handle the PID
}

#endif //PID_H
