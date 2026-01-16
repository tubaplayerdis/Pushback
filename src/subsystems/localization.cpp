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
#include "../../include/cls/localization_utils.hpp"
#include "../../include/pros/rtos.hpp"
#include <memory>
#include <chrono>
#include <cstring>
#include <optional>
#include <random>
#include <math.h>

#include "../../../../../../pros-toolchain/usr/arm-none-eabi/include/math.h"
#include "../../../../../../pros-toolchain/usr/arm-none-eabi/include/math.h"


std::unique_ptr<localization> odometry_instance;

using namespace ports::localization;
using namespace ports::localization::settings;


/**
 * @brief The distance to the wall from the center of the field.
 *
 * @details Initially it would seem as though the field is around 72 inches away as the field's center is described as 12ft x 12ft and therefore 144in x 144in,
 * but the reality is that 4 inches are taken by the field walls and subsequently the wall is 70.208 inches away from the center.
 */
static constexpr float wall_coord = 70.208;

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
    controller_settings_angular(2.35,  // kP — reduce a bit (was 1.6)
                                0.001,  // kI — keep off
                                15.9,  // kD — increase slightly for more damping
                                0.0,    // anti-windup
                                0.5,  // small error range
                                100,  // small error timeout
                                3,  // large error range
                                500,  // large error timeout
                                0     // slew rate
    );
}

static const localization_options loc_options = {};

/// timepoint value representing the last time the tick function was ran and updated this variable
static std::uint32_t time_at_last_call;

localization::localization() :
        inertial(INERTIAL),
        rotation_vertical(ROTATION_VERTICAL),
        tracking_vertical(&rotation_vertical, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_HORIZONTAL),
        odom_sensors(&tracking_vertical, nullptr, nullptr, nullptr, &inertial),
        lem_drivetrain(&drivetrain::get()->motors_left, &drivetrain::get()->motors_right, ports::drivetrain::settings::DRIVETRAIN_TRACK_WIDTH, ports::drivetrain::settings::DRIVETRAIN_WHEEL_DIAMETER, ports::drivetrain::settings::DRIVETRAIN_RPM, ports::drivetrain::settings::DRIVETRAIN_HORIZONTAL_DRIFT),
        lem_chassis(lem_drivetrain, pid::controller_settings_lateral, pid::controller_settings_angular, odom_sensors, &controller::expo_curve_throttle, &controller::expo_curve_steer),
        estimated_velocity(0,0,0),
        estimated_position(0,0,0),
        rear_loc(offsets::REAR, REAR_LOC),
        right_loc(offsets::RIGHT, LEFT_LOC),
        left_loc(offsets::LEFT, RIGHT_LOC),
        front_loc(offsets::FRONT, FRONT_LOC),
        l_chassis(loc_options, &inertial, &lem_chassis, {&rear_loc, &right_loc, &left_loc, &front_loc})
{
    time_at_last_call = pros::millis();
    lem_chassis.calibrate(true);
}

void localization::tick_implementation()
{
    constexpr auto FULL_POWER = 127;

    //Acquire throttle and turning values
    int32_t throttle = -1 * controller_master.get_analog(ports::drivetrain::controls::VERTICAL_AXIS);
    int32_t turn = controller_master.get_analog(ports::drivetrain::controls::HORIZONTAL_AXIS);

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
    l_chassis.reset_location();
}
