//
// Created by aaron on 7/21/2025.
//

#include "../include/monitor.h"
#include <vector>
#include <errno.h>

std::vector<std::function<void(const char *, int, const char *, char[256])>> ErrorCallbackFunctions = std::vector<std::function<void(const char *, int, const char *, char[256])>>();

void monitor::ReportError(const char *sFunction, int iLine, const char *sFileName)
{
    char cErrorMessage[256];
    strerror_s(cErrorMessage, sizeof(cErrorMessage), errno);
    for (auto function : ErrorCallbackFunctions)
    {
        function(sFunction, iLine, sFileName, cErrorMessage);
    }
}

void monitor::PerformAllChecks()
{
}

bool monitor::areConnectionsValid()
{
    return true;
}

bool monitor::areMotorsCool()
{
    return true;
}

void monitor::registerErrorCallback(const std::function<void(const char *, int, const char *, char[256])> &lambda)
{
    ErrorCallbackFunctions.push_back(lambda);
}

