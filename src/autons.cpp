//
// Created by aaron on 9/10/2025.
//

#include "../include/autons.h"
#include "../include/titanselect/titanselect.hpp"
#include "../include/subsystems/drivetrain.h"
#include "../include/subsystems/conveyor.h"
#include "../include/pros/adi.hpp"
#include "../include/pros/rtos.h"
#include "../include/pros/motors.hpp"

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
    lemlib::Chassis* chassis = &dt->lem_chassis;
    chassis->setPose(0, 0, 0);

    chassis->setPose(0, 0, 0);

    //chassis->turnToHeading(90, 3000);
    chassis->moveToPose(-0.34, -19.13, -1.00, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 30, .earlyExitRange = 3});
    chassis->waitUntilDone();
    chassis->moveToPose(-7.13, -56.53, 53.88, 1500, {.forwards = false});
    //0.34, -19.13, -0.136
    //-7.13, -56.53, 53.88
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
    conv->intake.move(-FULL_POWER);
    conv->conveyor_group.move(FULL_POWER);
    lemlib::Chassis* chassis = &dt->lem_chassis;


    nine_left_auton();

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