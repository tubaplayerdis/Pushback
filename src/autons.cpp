//
// Created by aaron on 9/10/2025.
//

#include "../include/autons.h"
#include "../include/titanselect/titanselect.hpp"
#include "../include/subsystems/drivetrain.h"
#include "../include/subsystems/conveyor.h"
#include "../Include/pros/adi.hpp"
#include "../include/pros/rtos.h"
#include "../include/pros/motors.hpp"
#include "stdio.h"
#include "math.h"
#include <math.h>

#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/iostream"
#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/fstream"
#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/iosfwd"
#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/iosfwd"

constexpr auto FULL_POWER = 127;

void testing_auton()
{
    std::ofstream deb = std::ofstream("/usd/deb.txt");
    lemlib::TurnToHeadingParams turnParams;
    turnParams.maxSpeed = 60;
    turnParams.minSpeed = 10;
    turnParams.direction = lemlib::AngularDirection::AUTO;
    turnParams.earlyExitRange = 0;

    drivetrain* dt = drivetrain::get();
    lemlib::Chassis* chassis = &dt->lem_chassis;

    chassis->setPose(0, 0, 0);
    //chassis->turnToHeading(90, 3000);
    chassis->moveToPose(0, 20, 0, 3000);

    while (true)
    {
        lemlib::Pose pose = chassis->getPose();
        controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
        pros::delay(100);
    }
}


/*
while (true)
{
    lemlib::Pose pose = chassis->getPose();
    controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
    pros::delay(100);
}
*/

void blue_left_auton()
{
    drivetrain* dt  = drivetrain::get();
    lemlib::Chassis* chassis = &dt->lem_chassis;
    chassis->setPose(0, 0, 0);
    chassis->turnToHeading(90, 3000);
}

void blue_right_auton()
{
    drivetrain* dt  = drivetrain::get();
}

// Definitions below
ts::auton autons::testing = ts::auton("testing", testing_auton);
ts::auton autons::blue_left = ts::auton("blue_left", blue_left_auton);
ts::auton autons::blue_right = ts::auton("blue_right", blue_right_auton);