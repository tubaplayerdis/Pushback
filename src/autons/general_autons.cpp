//
// Created by aaron on 9/10/2025.
//

#include "../../include/autons.h"
#include "../../include/titanselect/titanselect.hpp"
#include "../../include/subsystems/drivetrain.h"
#include "../../include/subsystems/conveyor.h"
#include "../../include/pros/adi.hpp"
#include "../../include/pros/misc.hpp"
#include "../../include/pros/rtos.hpp"
#include "../../include/pros/motors.hpp"

constexpr auto FULL_POWER = 127;

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

    while (true)
    {
        lemlib::Pose pose = chassis->getPose();
        controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
        pros::delay(50);
    }
}

void skills_auton()
{
    drivetrain* dt  = drivetrain::get();
    conveyor* conv = conveyor::get();
    lemlib::Chassis* chassis = &dt->lem_chassis;
    chassis->setPose(0, 0, 0);
    chassis->moveToPose(0, 20, 0, 3000, {}, false);
    {   //Start intake and conveyor
        conv->intake.move(FULL_POWER);
        conv->conveyor_group.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
    }
    chassis->tank(-70, -70, true);
    pros::delay(1700);
    chassis->tank(0,0,true);
}
//-5.59, -53.15, 20.25

// Definitions below
ts::auton autons::testing = ts::auton("Dogs Out", testing_auton);
ts::auton autons::skills = ts::auton("Skills", skills_auton);