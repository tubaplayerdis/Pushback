//
// Created by aaron on 7/20/2025.
//

#ifndef MONITOR_H
#define MONITOR_H

#include <functional>

//Custom error handler macro
#define Handle(exp) if((exp) == PROS_ERR) monitor::ReportError(__FUNCTION__, __LINE__, __FILE__)

namespace monitor
{
    void ReportError(const char* sFunction, int iLine, const char* sFileName);

    void PerformAllChecks();

    //Individual Checks
    bool areConnectionsValid();
    bool areMotorsCool();

    void registerErrorCallback(const std::function<void(const char*, int, const char*, char[256])>& lambda);
}

#endif //MONITOR_H
