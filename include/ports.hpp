//
// Created by aaron on 8/30/2025.
//

#ifndef PUSHBACK_PORTS_HPP
#define PUSHBACK_PORTS_HPP

#include "pros/misc.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"

/// All ports and constants are defined in this file for simplicity.

namespace ports
{
    constexpr auto CYCLE_AUTONS = pros::E_CONTROLLER_DIGITAL_DOWN;

    namespace tune
    {
        constexpr auto PID_TUNE_MODE = pros::E_CONTROLLER_DIGITAL_A;
        constexpr auto SWAP_MODES = pros::E_CONTROLLER_DIGITAL_B;
        constexpr auto KP_UP = pros::E_CONTROLLER_DIGITAL_UP;
        constexpr auto KP_DOWN = pros::E_CONTROLLER_DIGITAL_DOWN;
        constexpr auto KD_UP = pros::E_CONTROLLER_DIGITAL_RIGHT;
        constexpr auto KD_DOWN = pros::E_CONTROLLER_DIGITAL_LEFT;
        constexpr auto TEST_ANGULAR = pros::E_CONTROLLER_DIGITAL_Y;
        constexpr auto TEST_LATERAL = pros::E_CONTROLLER_DIGITAL_X;
    }

    namespace conveyor
    {
        constexpr auto EXHAUST = 14;
        constexpr auto CONVEYOR = -17;
        constexpr auto MATCH_LOADER = 'B';
        constexpr auto WINGS = 'D';
        constexpr auto TRAPDOOR = 'C';
        constexpr auto DESCORE = 'A';

        namespace controls
        {
            constexpr auto CONVEYOR_IN = pros::E_CONTROLLER_DIGITAL_L1;
            constexpr auto CONVEYOR_OUT = pros::E_CONTROLLER_DIGITAL_R1;
            constexpr auto TOGGLE_DESCORE = pros::E_CONTROLLER_DIGITAL_Y;
            constexpr auto EXHAUST_OUT = pros::E_CONTROLLER_DIGITAL_L2;
            constexpr auto TOGGLE_MATCH_LOADER = pros::E_CONTROLLER_DIGITAL_X;
            constexpr auto TOGGLE_WINGS = pros::E_CONTROLLER_DIGITAL_A;
            constexpr auto RAMP_MACRO = pros::E_CONTROLLER_DIGITAL_R2;//Moves the ramp down, move intake in and ove exhaust backwards slightly
            constexpr auto HALF_OUT = pros::E_CONTROLLER_DIGITAL_RIGHT;
            constexpr auto QUARTER_OUT = pros::E_CONTROLLER_DIGITAL_LEFT;
        }
    }

    namespace localization
    {
        constexpr auto INERTIAL = 7;
        constexpr auto ROTATION_VERTICAL = 18;
        constexpr auto ENCODER_HORIZONTAL_0 = 'E';
        constexpr auto ENCODER_HORIZONTAL_1 = 'F';
        constexpr auto FRONT_LOC = 19; //North
        constexpr auto REAR_LOC = 4; //South
        constexpr auto LEFT_LOC = 3; //West
        constexpr auto RIGHT_LOC = 8; //East

        constexpr auto LEFT_A = -9;//9, 15, 10
        constexpr auto LEFT_B = -15;
        constexpr auto LEFT_C = -10;
        constexpr auto RIGHT_A = 1;
        constexpr auto RIGHT_B = 2;
        constexpr auto RIGHT_C = 16;

        namespace settings
        {
            constexpr auto ODOMETRY_DIST_FROM_CENTER_HORIZONTAL = -0.87; //The vertical offset of the horizontal tracking wheel from the center of the robot in inches.
            constexpr auto ODOMETRY_DIST_FROM_CENTER_VERTICAL = 0.0; //The horizontal offset of the vertical tracking wheel from the center of the robot in inches.
            constexpr auto ODOMETRY_WHEEL_SIZE_VERTICAL = lemlib::Omniwheel::NEW_275;
            constexpr auto ODOMETRY_WHEEL_SIZE_HORIZONTAL = lemlib::Omniwheel::NEW_275;
            constexpr auto INERTIAL_DRIFT = 1.00395;//1.0076;

            constexpr auto DRIVETRAIN_TRACK_WIDTH = 10.5;
            constexpr auto DRIVETRAIN_WHEEL_DIAMETER = lemlib::Omniwheel::NEW_325;
            constexpr auto DRIVETRAIN_RPM = 450;
            constexpr auto DRIVETRAIN_HORIZONTAL_DRIFT = 2;
            constexpr auto DRIVETRAIN_MOTOR_CARTRIDGE = pros::v5::MotorGears::blue;
        }

        namespace controls
        {
            constexpr auto VERTICAL_AXIS = pros::E_CONTROLLER_ANALOG_LEFT_Y;
            constexpr auto HORIZONTAL_AXIS = pros::E_CONTROLLER_ANALOG_RIGHT_X;
            constexpr auto BARRIER_CROSS = pros::E_CONTROLLER_DIGITAL_B;
        }

        namespace offsets
        {
            constexpr auto FRONT_X = 4.75;
            constexpr auto FRONT_Y = -4.625;

            constexpr auto RIGHT_X = 5.25;
            constexpr auto RIGHT_Y = -1;

            constexpr auto REAR_X = 5.823;
            constexpr auto REAR_Y = 4.691;

            constexpr auto LEFT_X = 5.25;
            constexpr auto LEFT_Y = -1;
        }
    }
}


#endif //PUSHBACK_PORTS_HPP
