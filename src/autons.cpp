//
// Created by aaron on 9/10/2025.
//

#include "../include/autons.h"
#include "../include/titanselect/titanselect.hpp"
#include "../include/subsystems/drivetrain.h"

void testing_auton()
{
    drivetrain* dt  = drivetrain::get();
    dt->lem_chassis.calibrate();
    dt->lem_chassis.moveToPoint(100, 100, 1000);
}

void blue_left_auton()
{
    drivetrain* dt  = drivetrain::get();
    dt->lem_chassis.calibrate();
}

void blue_right_auton()
{
    drivetrain* dt  = drivetrain::get();
    dt->lem_chassis.calibrate();
}

// Definitions below
ts::auton autons::testing = ts::auton("testing", testing_auton);
ts::auton autons::blue_left = ts::auton("blue_left", blue_left_auton);
ts::auton autons::blue_right = ts::auton("blue_right", blue_right_auton);