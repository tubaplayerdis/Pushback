//
// Created by aaron on 9/10/2025.
//

#include "../include/autons.h"
#include "../include/titanselect/titanselect.hpp"
#include "../include/subsystems/drivetrain.h"
#include "../include/subsystems/conveyor.h"
#include "../include/pros/adi.hpp"
#include "../include/pros/misc.hpp"
#include "../include/pros/rtos.hpp"
#include "../include/pros/motors.hpp"
#include "stdio.h"
#include "math.h"
#include <math.h>

#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/iostream"
#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/fstream"
#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/iosfwd"
#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/iosfwd"

constexpr auto FULL_POWER = 127;

void nine_left_auton()
{
    drivetrain* dt  = drivetrain::get();
    lemlib::Chassis* chassis = &dt->lem_chassis;
    chassis->setPose(0, 0, 0);
    chassis->turnToHeading(90, 3000);
}

void nine_right_auton()
{
    drivetrain* dt  = drivetrain::get();
    conveyor* conv = conveyor::get();
    lemlib::Chassis* chassis = &dt->lem_chassis;
    chassis->setPose(0, 0, 0);

    conv->intake.move(-FULL_POWER);
    conv->conveyor_group.move(FULL_POWER);
    conv->exhaust.move(-0.3 * FULL_POWER);

    chassis->moveToPose(-14.00, -25.26, 42.70, 1400, {.forwards = false, .maxSpeed = 65 , .minSpeed = 30}, false);
    //                    -35.91
    chassis->moveToPose(-35.96, -43.93, 51.88, 1600, {.forwards = false, .maxSpeed = 65, .minSpeed = 30, .earlyExitRange = 1}, false);
    conveyor::get()->lift.toggle();
    pros::delay(500);
    chassis->moveToPose(-28.00, -13.00, 0, 1500, {.minSpeed = 30}, false);
    chassis->moveToPose(-43.71, -24.00, 180.0, 1500, {.earlyExitRange = 0}, true);
    {
        conv->conveyor_group.move(-FULL_POWER);
        pros::delay(600);
    }
    chassis->waitUntilDone();
    chassis->tank(30,30, true);
    conv->conveyor_group.move(FULL_POWER);
    conv->exhaust.move(FULL_POWER);
    conv->intake.move(-FULL_POWER);
    pros::delay(2600);
    conv->conveyor_group.brake();
    conv->exhaust.brake();
    conv->intake.brake();
    chassis->moveToPose(-42.50, 4.90, 180, 1500, {.forwards = false, .minSpeed = 50}, false);
    chassis->tank(-55,-55, true);
    conv->intake.move(-FULL_POWER);
    conv->conveyor_group.move(FULL_POWER);
    conv->exhaust.move(-0.3 * FULL_POWER);
    pros::delay(1000);

    //Final Score.
    chassis->moveToPose(-43.71, -24.00, 180.0, 1500, {.earlyExitRange = 0}, false);
    chassis->tank(30,30, true);
    conv->conveyor_group.move(FULL_POWER);
    conv->exhaust.move(FULL_POWER);
    conv->intake.move(-FULL_POWER);
    pros::delay(1400);
    chassis->tank(-100,-100, true);
    pros::delay(300);
    chassis->tank(100, 100, true);

    //0.34, -19.13, -0.136
    //-7.13, -56.53, 53.88
    //-42.71 -23.33, 180

    //Match Loader: -44.20, 5.9, 180
}

void testing_auton()
{
    lemlib::TurnToHeadingParams turnParams;
    turnParams.maxSpeed = 60;
    turnParams.minSpeed = 10;
    turnParams.direction = lemlib::AngularDirection::AUTO;
    turnParams.earlyExitRange = 0;

    drivetrain* dt = drivetrain::get();
    conveyor* conv = conveyor::get();
    lemlib::Chassis* chassis = &dt->lem_chassis;


    nine_right_auton();

    while (true)
    {
        lemlib::Pose pose = chassis->getPose();
        controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
        pros::delay(50);
    }
}
//-5.59, -53.15, 20.25

// Definitions below
ts::auton autons::testing = ts::auton("testing", testing_auton);
ts::auton autons::nine_left = ts::auton("9 Left", nine_left_auton);
ts::auton autons::nine_right = ts::auton("9 Right", nine_right_auton);
ts::auton autons::nine_awp_low = ts::auton("9 AWP Low", testing_auton);
ts::auton autons::nine_awp_high = ts::auton("9 AWP High", testing_auton);