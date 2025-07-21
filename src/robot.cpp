//
// Created by aaron on 7/20/2025.
//
#include "../include/robot.h"

bool robot::CalibrateRobot()
{
    return true;
}

void robot::InitializeTasks()
{

}

void robot::TickSubsystems()
{
    Conveyor.Tick();
}

void robot::drivetrain::drivetrainControl()
{
    int32_t leftY = Controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int32_t rightX = Controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    Chassis.arm(leftY, rightX);
}

