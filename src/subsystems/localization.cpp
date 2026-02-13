//
// Created by aaron on 8/25/2025.
//

#include "../../include/subsystems/localization.hpp"

#include <array>

#include "../../include/ports.hpp"
#include "../../include/pros/imu.hpp"
#include "../../include/pros/imu.h"
#include "../../include/controller.hpp"
#include "../../include/lemlib/pose.hpp"
#include "../../include/subsystems/drivetrain.hpp"
#include "../../include/units/units.hpp"
#include "../../include/TitanReset/TitanReset.hpp"
#include "../../include/pros/rtos.hpp"
#include <memory>
#include <chrono>
#include <cstring>
#include <optional>
#include <random>
#include <math.h>

std::unique_ptr<localization> odometry_instance;

namespace pid
{
    // Linear/lateral movement settings
    lemlib::ControllerSettings
    controller_settings_lateral(14.01, // proportional gain (kP)
                                              0.00, // integral gain (kI)
                                              75, // derivative gain (kD)
                                              0, // anti windup
                                              0.5, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
    );

    // Angular/turning settings
    lemlib::ControllerSettings
    controller_settings_angular(2.5,  // kP — reduce a bit (was 1.6)
                                0.001,  // kI — keep off
                                15.8,  // kD — increase slightly for more damping
                                0.0,    // anti-windup
                                0.5,  // small error range
                                100,  // small error timeout
                                3,  // large error range
                                500,  // large error timeout
                                0     // slew rate
    );
}

using namespace ports::localization;
using namespace ports::localization::settings;

localization::localization() :
        inertial(INERTIAL),
        rotation_vertical(ROTATION_VERTICAL),
        encoder_horizontal(ENCODER_HORIZONTAL_0, ENCODER_HORIZONTAL_1, true),
        tracking_vertical(&rotation_vertical, ODOMETRY_WHEEL_SIZE_VERTICAL, ODOMETRY_DIST_FROM_CENTER_VERTICAL),
        tracking_horizontal(&encoder_horizontal, ODOMETRY_WHEEL_SIZE_HORIZONTAL, ODOMETRY_DIST_FROM_CENTER_HORIZONTAL),
        odom_sensors(&tracking_vertical, nullptr, &tracking_horizontal, nullptr, &inertial),
        lem_drivetrain(&drivetrain::get()->motors_left, &drivetrain::get()->motors_right, ports::drivetrain::settings::DRIVETRAIN_TRACK_WIDTH, ports::drivetrain::settings::DRIVETRAIN_WHEEL_DIAMETER, ports::drivetrain::settings::DRIVETRAIN_RPM, ports::drivetrain::settings::DRIVETRAIN_HORIZONTAL_DRIFT),
        lem_chassis(lem_drivetrain, pid::controller_settings_lateral, pid::controller_settings_angular, odom_sensors, &controller::expo_curve_throttle, &controller::expo_curve_steer),
        rear_loc({offsets::REAR_X, offsets::REAR_Y}, REAR_LOC),
        right_loc({offsets::RIGHT_X, offsets::RIGHT_Y}, LEFT_LOC),
        left_loc({offsets::LEFT_X, offsets::LEFT_Y}, RIGHT_LOC),
        front_loc({offsets::FRONT_X, offsets::FRONT_Y}, FRONT_LOC),
        l_chassis(&inertial, &lem_chassis, {&rear_loc, &right_loc, &left_loc, &front_loc})
{
    lem_chassis.calibrate(true);
}

void localization::tick_implementation()
{
    constexpr auto FULL_POWER = 127;

    //Acquire throttle and turning values
    int32_t throttle = -1 * controller_master.get_analog(ports::drivetrain::controls::VERTICAL_AXIS);
    int32_t turn = controller_master.get_analog(ports::drivetrain::controls::HORIZONTAL_AXIS);

    if (throttle == 0 && turn == 0)
    {
        drivetrain* drive = drivetrain::get();
        if (controller_master.get_digital(ports::drivetrain::controls::SWING_LEFT))
        {
            (void)drive->motors_left.set_brake_mode_all(pros::MotorBrake::hold);
            (void)drive->motors_right.move(FULL_POWER);
        }
        else if (controller_master.get_digital(ports::drivetrain::controls::SWING_RIGHT))
        {
            (void)drive->motors_left.move(FULL_POWER);
            (void)drive->motors_right.set_brake_mode_all(pros::MotorBrake::hold);
        }
        else if (controller_master.get_digital(ports::drivetrain::controls::BARRIER_CROSS))
        {
            lem_chassis.tank(-71, -68, true);
        }
        else
        {
            if (drive->motors_left.get_brake_mode(0) != pros::MotorBrake::coast || drive->motors_right.get_brake_mode(0) != pros::MotorBrake::coast)
            {
                (void)drive->motors_left.set_brake_mode_all(pros::MotorBrake::coast);
                (void)drive->motors_right.set_brake_mode_all(pros::MotorBrake::coast);
            }
            (void)drive->motors_left.brake();
            (void)drive->motors_right.brake();
        }
        return;
    }

    //Apply inputs.
    lem_chassis.arcade(throttle, turn);
}

localization* localization::get()
{
    if (!odometry_instance) odometry_instance = std::unique_ptr<localization>(new localization() );
    return odometry_instance.get();
}

void localization::distance_sensor_reset()
{
    l_chassis.perform_dsr();
}
