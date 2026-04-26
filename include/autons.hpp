//
// Created by aaron on 7/24/2025.
//

#ifndef AUTONS_H
#define AUTONS_H

#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"
#include "titanselect/titanselect.hpp"

//Macro that expands the pos struct for use with the moveToPose command
#define POS(pos) pos.X, pos.Y, pos.T

//Macro that expands the pos struct for use with the moveToPoint, swingToPoint, turnToPoint commands
#define MPOS(pos) pos.X, pos.Y

//Macro that expands the pos struct for use with the turnToHeading, swingToHeading commands
#define TPOS(pos) pos.T

//#define SKILLS

struct pos
{
    const float X;
    const float Y;
    const float T;

    pos(const float x, const float y, const float t) : X(x), Y(y), T(t) {}
};

void skills_routine();
void quick_approach_right(lemlib::Chassis* chassis);

inline void wing_align(lemlib::Chassis* chassis)
{
    chassis->turnToHeading(150, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 500, {}, false);
}

inline void brake_chassis(lemlib::Chassis* chassis)
{
    chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    (void)chassis->drivetrain.leftMotors->brake();
    (void)chassis->drivetrain.rightMotors->brake();
}

inline void wing_movement(lemlib::Chassis* chassis, const pos &end_spot)
{
    wing_align(chassis);
    chassis->moveToPoint(MPOS(end_spot), 1500, {.minSpeed = 40}, false);
    brake_chassis(chassis);
}

namespace autons
{
    extern ts::auton skills;
    extern ts::auton sawp_dsr;
    extern ts::auton sawp_dsr_push;
    extern ts::auton sawp_dsr_counter;
    extern ts::auton sawp_dsr_counter_push;
    extern ts::auton sawp_move;
    extern ts::auton elims_left_dsr;
    extern ts::auton elims_left_fast;
    extern ts::auton elims_left_fast_fast;
    extern ts::auton elims_left_middle_end;
    extern ts::auton elims_left_middle_wing;
    extern ts::auton elims_left_middle_wing_fast;
    extern ts::auton elims_right_middle_fast;
    extern ts::auton elims_right_middle_slow;
    extern ts::auton elims_right;
    extern ts::auton elims_right_middle_low;
}


#endif //AUTONS_H
