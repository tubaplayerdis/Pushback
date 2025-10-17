//
// Created by aaron on 9/10/2025.
//

#include "../include/autons.h"
#include "../include/titanselect/titanselect.hpp"
#include "../include/subsystems/drivetrain.h"
#include "../include/subsystems/conveyor.h"
#include "../Include/pros/adi.hpp"
#include "../include/pros/motors.hpp"
#include "stdio.h"

constexpr auto FULL_POWER = 127;

//Autons should set color sorting color for driver control.

void testing_auton()
{
    odometry* od = odometry::get();
    drivetrain* dt  = drivetrain::get();
    conveyor* conv  = conveyor::get();
    lemlib::Chassis* chassis = &dt->lem_chassis;
    chassis->setPose(0,0,0);
    //Look at notebook for rough guesstimations of auton locations
    //(void)conv->conveyor_group.move(FULL_POWER);
    //(void)conv->intake.move(-FULL_POWER);
    //chassis->turnToHeading(90, 1000);
    //chassis->swingToHeading(90, lemlib::DriveSide::LEFT,8000);
    //chassis->moveToPose(0.0, 26, /*16.9*/0, 5000, {.maxSpeed = 30});
    //2.12, 19.77, 16.9


    //TODO: Run test where one side is set to 100 power for 0.5 seconds and compare amount driven

    while (true)
    {
        lemlib::Pose pose = chassis->getPose();
        controller_master.print(1,0, "%.2f, %.2f, %.2f", dt->motors_left.get_position(0), dt->motors_right.get_position(2), pose.theta);
        printf("L0 %.2f L1 %.2f L2 %.2f | R0 %.2f R1 %.2f R2 %.2f\n",
        dt->motors_left.get_position(0),
        dt->motors_left.get_position(1),
        dt->motors_left.get_position(2),
        dt->motors_right.get_position(0),
        dt->motors_right.get_position(1),
        dt->motors_right.get_position(2));
    }



    //chassis->moveToPose(8,8,0,600);
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