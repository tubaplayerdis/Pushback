//
// Created by aaron on 9/10/2025.
//

#include "../include/autons.h"
#include "../include/titanselect/titanselect.hpp"
#include "../include/subsystems/drivetrain.h"

//Autons should set color sorting color for driver control.

void testing_auton()
{
    odometry* od = odometry::get();
    drivetrain* dt  = drivetrain::get();
    lemlib::Chassis* chassis = &dt->lem_chassis;
    chassis->setPose(0,0,0);
    chassis->moveToPoint(8, 0, 400);
}

void blue_left_auton()
{
    drivetrain* dt  = drivetrain::get();
}

void blue_right_auton()
{
    drivetrain* dt  = drivetrain::get();
}

// Definitions below
ts::auton autons::testing = ts::auton("testing", testing_auton);
ts::auton autons::blue_left = ts::auton("blue_left", blue_left_auton);
ts::auton autons::blue_right = ts::auton("blue_right", blue_right_auton);