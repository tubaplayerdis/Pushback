//
// Created by aaron on 7/21/2025.
//

#include "../include/monitor.h"
#include <vector>
#include <stdio.h>
#include <errno.h>
#include <system_error>
#include "../include/drivetrain.h"
#include "../include/conveyor.h"

std::vector<std::function<void(const char *, int, const char *, const char*)>> ErrorCallbackFunctions = std::vector<std::function<void(const char *, int, const char *, const char*)>>();

void monitor::ReportError(const char *sFunction, int iLine, const char *sFileName)
{
    std::error_code ec(errno, std::generic_category());
    for (auto function : ErrorCallbackFunctions)
    {
        function(sFunction, iLine, sFileName, ec.message().c_str());
    }
}

bool monitor::MotorsCool()
{
    //Left side DT Motors
    for (int i = 0; i < Drivetrain->MotorsLeft.size(); i++)
    {
        if (Drivetrain->MotorsLeft.get_temperature(i) >= MOTOR_TEMPERATURE_OVERHEAT_THRESHOLD_C) return false;
    }

    //Right side DT Motors
    for (int i = 0; i < Drivetrain->MotorsRight.size(); i++)
    {
        if (Drivetrain->MotorsRight.get_temperature(i) >= MOTOR_TEMPERATURE_OVERHEAT_THRESHOLD_C) return false;
    }

    //Conveyor normal group
    for (int i = 0; i < Conveyor->NormalGroup.size(); i++)
    {
        if (Conveyor->NormalGroup.get_temperature(i) >= MOTOR_TEMPERATURE_OVERHEAT_THRESHOLD_C) return false;
    }

    //Conveyor inverted group
    for (int i = 0; i < Conveyor->InvertedGroup.size(); i++)
    {
        if (Conveyor->InvertedGroup.get_temperature(i) >= MOTOR_TEMPERATURE_OVERHEAT_THRESHOLD_C) return false;
    }

    return true;
}


void monitor::registerErrorCallback(const std::function<void(const char *, int, const char *, const char *)> &lambda)
{
    ErrorCallbackFunctions.push_back(lambda);
}

