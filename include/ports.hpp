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


    namespace drivetrain
    {
        constexpr auto LEFT_A = -18;
        constexpr auto LEFT_B = -19;
        constexpr auto LEFT_C = -20;
        constexpr auto RIGHT_A = 11;
        constexpr auto RIGHT_B = 12;
        constexpr auto RIGHT_C = 13;

        namespace controls
        {
            constexpr auto VERTICAL_AXIS = pros::E_CONTROLLER_ANALOG_LEFT_Y;
            constexpr auto HORIZONTAL_AXIS = pros::E_CONTROLLER_ANALOG_RIGHT_X;
            constexpr auto SWING_LEFT = pros::E_CONTROLLER_DIGITAL_LEFT;
            constexpr auto SWING_RIGHT = pros::E_CONTROLLER_DIGITAL_RIGHT;
        }

        namespace settings
        {
            constexpr auto DRIVETRAIN_TRACK_WIDTH = 10.5;
            constexpr auto DRIVETRAIN_WHEEL_DIAMETER = lemlib::Omniwheel::NEW_325;
            constexpr auto DRIVETRAIN_RPM = 450;
            constexpr auto DRIVETRAIN_HORIZONTAL_DRIFT = 8;
            constexpr auto DRIVETRAIN_MOTOR_CARTRIDGE = pros::v5::MotorGears::blue;
        }
    }

    namespace conveyor
    {
        constexpr auto EXHAUST = 1;
        constexpr auto CONVEYOR = -10;
        constexpr auto MATCH_LOADER = 'B';
        constexpr auto WINGS = 'A';
        constexpr auto TRAPDOOR = 'C';

        namespace controls
        {
            constexpr auto CONVEYOR_IN = pros::E_CONTROLLER_DIGITAL_L1;
            constexpr auto CONVEYOR_OUT = pros::E_CONTROLLER_DIGITAL_R1;
            constexpr auto EXHAUST_OUT = pros::E_CONTROLLER_DIGITAL_L2;
            constexpr auto TOGGLE_MATCH_LOADER = pros::E_CONTROLLER_DIGITAL_X;
            constexpr auto TOGGLE_WINGS = pros::E_CONTROLLER_DIGITAL_UP;
            constexpr auto RAMP_MACRO = pros::E_CONTROLLER_DIGITAL_R2;//Moves the ramp down, move intake in and ove exhaust backwards slightly
        }
    }

    namespace localization
    {
        constexpr auto INERTIAL = 8;
        constexpr auto ROTATION_VERTICAL = -9;
        constexpr auto FRONT_LOC = 4; //North
        constexpr auto REAR_LOC = 3; //South
        constexpr auto LEFT_LOC = 2; //West
        constexpr auto RIGHT_LOC = 5; //East

        namespace settings
        {
            constexpr auto ODOMETRY_DIST_FROM_CENTER_HORIZONTAL = 0; //The horizontal offset of the tracking wheel from the center of the robot in inches.
            constexpr auto ODOMETRY_WHEEL_SIZE = lemlib::Omniwheel::NEW_2;
        }

        namespace offsets
        {
            constexpr auto FRONT = 5.5;
            constexpr auto REAR = 5.85;
            constexpr auto LEFT = 5;
            constexpr auto RIGHT = 5;
        }
    }
}


#endif //PUSHBACK_PORTS_HPP
