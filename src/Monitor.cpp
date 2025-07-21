//
// Created by aaron on 7/21/2025.
//

#include "../include/Monitor.h"
#include <vector>
#include <errno.h>

std::vector<std::function<void(const char *, int, const char *, const char *)>> ErrorCallbackFunctions = std::vector<std::function<void(const char *, int, const char *, const char *)>>();

void Monitor::ReportError(const char *sFunction, int iLine, const char *sFileName)
{
    char* cErrorMessage = strerror(errno);
    for (auto function : ErrorCallbackFunctions)
    {
        function(sFunction, iLine, sFileName, cErrorMessage);
    }
}

void Monitor::PerformAllChecks()
{
}

bool Monitor::areConnectionsValid()
{
    return true;
}

bool Monitor::areMotorsCool()
{
    return true;
}

void Monitor::registerErrorCallback(const std::function<void(const char *, int, const char *, const char *)> &lambda)
{
    ErrorCallbackFunctions.push_back(lambda);
}

