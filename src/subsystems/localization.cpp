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

namespace field_constants
{
    constexpr float metal = 70.408;
    constexpr float plastic = 70.283;
    constexpr float home = 70.208;
}

namespace pid
{
    // Linear/lateral movement settings
    lemlib::ControllerSettings
    controller_settings_lateral(11.00, // proportional gain (kP)
                                              0.00, // integral gain (kI)
                                              68.25, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
    );

    // Angular/turning settings
    lemlib::ControllerSettings
    controller_settings_angular(4.5,   // kP: Lowered slightly to reduce 180° momentum
                            0.0,  // kI: Increased to help finish small 10° turns
                            34.61,  // kD: Lowered slightly to reduce "choke" on start
                            0.0,   // anti-windup: Prevents kI from growing too large
                            1, // small error range, in inches
                            100, // small error range timeout, in milliseconds
                            3, // large error range, in inches
                            500, // large error range timeout, in milliseconds
                            0      // slew rate
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
        l_chassis(&inertial, &lem_chassis, {&front_loc, &right_loc, &rear_loc, &left_loc}, field_constants::home)
{
    lem_chassis.calibrate(true);
}

void localization::tick_implementation()
{
    constexpr auto FULL_POWER = 127;

    //Acquire throttle and turning values
    int32_t throttle = -1 * controller_master.get_analog(ports::drivetrain::controls::VERTICAL_AXIS);
    int32_t turn = controller_master.get_analog(ports::drivetrain::controls::HORIZONTAL_AXIS);

    if (throttle == 0 && turn == 0 && controller_master.get_digital(ports::drivetrain::controls::BARRIER_CROSS))
    {
        lem_chassis.tank(-55, -52, true);
    }
    else
    {
        //Apply inputs.
        lem_chassis.arcade(throttle, turn, false, 0.65);
    }
}

localization* localization::get()
{
    if (!odometry_instance) odometry_instance = std::unique_ptr<localization>(new localization() );
    return odometry_instance.get();
}
