//
// Created by aaron on 7/20/2025.
//

#ifndef ROBOT_H
#define ROBOT_H

#include "pros/rtos.hpp"

    //Handle various subsystems and controls/monitors
    namespace tasks
    {
        pros::Task* ControllerGraphics();
        pros::Task* Monitor();
    }

#endif //ROBOT_H
