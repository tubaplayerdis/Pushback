//
// Created by aaron on 7/20/2025.
//
#include "../include/Robot.h"

bool robot::calibrateRobot()
{
    return true;
}

void robot::initializeTasks()
{

}

void robot::conveyor::conveyorControl()
{
    if (Controller.get_digital(CONVEYOR_IN))
    {
        conveyor::motors::NormalGroup.move(FULL_POWER_FWD);
        conveyor::motors::InvertedGroup.move(-FULL_POWER_FWD);
    } else if (Controller.get_digital(CONVEYOR_OUT))
    {

    } else
    {

    }
}

void robot::drivetrain::drivetrainControl()
{
    int32_t leftY = Controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int32_t rightX = Controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    Chassis.arm(leftY, rightX);
}

