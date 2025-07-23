//
// Created by aaron on 7/20/2025.
//

#ifndef MONITOR_H
#define MONITOR_H

#include <functional>

//Handles PROS_ERR throwing functions using proprietary error handler
#define Handle(exp) if((exp) == PROS_ERR) monitor::ReportError(__FUNCTION__, __LINE__, __FILE__)

//Temp at which the motors overheat in C
#define MOTOR_TEMPERATURE_OVERHEAT_THRESHOLD_C 55.0

namespace monitor
{
    void ReportError(const char* sFunction, int iLine, const char* sFileName);

    bool MotorsCool();

    //void registerErrorCallback(const std::function<void(const char*, int, const char*, const char*)>& lambda);
}

#endif //MONITOR_H
