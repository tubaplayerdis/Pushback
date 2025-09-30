//
// Created by aaron on 8/30/2025.
//

#ifndef PUSHBACK_PORTS_H
#define PUSHBACK_PORTS_H

#include "pros/misc.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

/// All ports and constants are defined in this file for simplicity.

namespace ports
{
    namespace drivetrain
    {
        constexpr auto LEFT_A = 20;
        constexpr auto LEFT_B = 1;
        constexpr auto LEFT_C = 6;
        constexpr auto RIGHT_A = 11;
        constexpr auto RIGHT_B = 9;
        constexpr auto RIGHT_C = 10;

        namespace controls
        {
            constexpr auto VERTICAL_AXIS = pros::E_CONTROLLER_ANALOG_LEFT_Y;
            constexpr auto HORIZONTAL_AXIS = pros::E_CONTROLLER_ANALOG_RIGHT_X;
        }

        namespace settings
        {
            constexpr auto DRIVETRAIN_TRACK_WIDTH = 10;
            constexpr auto DRIVETRAIN_WHEEL_DIAMETER = lemlib::Omniwheel::NEW_325;
            constexpr auto DRIVETRAIN_RPM = 450;
            constexpr auto DRIVETRAIN_HORIZONTAL_DRIFT = 2;
            constexpr auto DRIVETRAIN_MOTOR_CARTRIDGE = pros::v5::MotorGears::blue;
        }
    }

    namespace conveyor
    {
        constexpr auto INTAKE = 8;
        constexpr auto EXHAUST = 5;
        constexpr auto CONVEYOR_A = 2;
        constexpr auto CONVEYOR_B = -3;
        constexpr auto LIFT = 'C';
        constexpr auto WINGS = 'A';
        constexpr auto RAMP = 'B';
        constexpr auto SPLITTER_OPTICAL = 0;

        namespace controls
        {
            constexpr auto CONVEYOR_IN = pros::E_CONTROLLER_DIGITAL_L1;
            constexpr auto CONVEYOR_OUT = pros::E_CONTROLLER_DIGITAL_R1;
            constexpr auto EXHAUST_OUT = pros::E_CONTROLLER_DIGITAL_L2;
            constexpr auto EXHAUST_IN = pros::E_CONTROLLER_DIGITAL_LEFT;
            constexpr auto TOGGLE_LIFT = pros::E_CONTROLLER_DIGITAL_X;
            constexpr auto TOGGLE_WINGS = pros::E_CONTROLLER_DIGITAL_UP;
            constexpr auto RAMP_MACRO = pros::E_CONTROLLER_DIGITAL_R2;//Moves the ramp down, move intake in and ove exhaust backwards slightly
            constexpr auto OVERRIDE_RAMP_UP = pros::E_CONTROLLER_DIGITAL_A;
            constexpr auto OVERRIDE_RAMP_DOWN = pros::E_CONTROLLER_DIGITAL_B;
        }
    }

    namespace odometry
    {
        constexpr auto INERTIAL = 14;
        constexpr auto ROTATION_VERTICAL = -13;
        constexpr auto ROTATION_HORIZONTAL = -8;

        namespace settings
        {
            constexpr auto ODOMETRY_DIST_FROM_CENTER_HORIZONTAL = -1.5;
            constexpr auto ODOMETRY_DIST_FROM_CENTER_VERTICAL = 0;
            constexpr auto ODOMETRY_WHEEL_SIZE = lemlib::Omniwheel::NEW_2;
        }
    }
}


#endif //PUSHBACK_PORTS_H
