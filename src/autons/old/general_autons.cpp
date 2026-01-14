//
// Created by aaron on 9/10/2025.
//

#include "../../../include/autons.hpp"
#include "../../../include/titanselect/titanselect.hpp"
#include "../../../include/subsystems/drivetrain.hpp"
#include "../../../include/subsystems/conveyor.hpp"
#include "../../../include/pros/adi.hpp"
#include "../../../include/pros/misc.hpp"
#include "../../../include/pros/rtos.hpp"
#include "../../../include/pros/motors.hpp"

constexpr auto FULL_POWER = 127;

void testing_auton()
{
    lemlib::TurnToHeadingParams turnParams;
    turnParams.maxSpeed = 60;
    turnParams.minSpeed = 10;
    turnParams.direction = lemlib::AngularDirection::AUTO;
    turnParams.earlyExitRange = 0;

    localization* dt = localization::get();
    conveyor* conv = conveyor::get();
    lemlib::Chassis* chassis = &dt->lem_chassis;

    chassis->setPose(0,0,0);

    chassis->moveToPoint(0, 10, 3000, {}, false);

    while (true)
    {
        lemlib::Pose pose = chassis->getPose();
        controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
        pros::Task::delay(50);
    }
}

void old_skills_auton()
{
    localization* dt  = localization::get();
    conveyor* conv = conveyor::get();
    lemlib::Chassis* chassis = &dt->lem_chassis;
    chassis->setPose(0, 0, 0);
    {   //Start intake and conveyor
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
    }
    conv->match_loader.toggle();
    pros::Task::delay(700);
    chassis->tank(-100, -100, true);
    pros::Task::delay(650);
    chassis->tank(0,0,true);
    pros::Task::delay(5000);

    {   //Start intake and conveyor
        conv->conveyor_intake.brake();
        conv->exhaust.brake();
    }

}
//-5.59, -53.15, 20.25

// Definitions below
ts::auton testing = ts::auton("Testing", testing_auton);