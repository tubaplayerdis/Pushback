//
// Created by aaron on 8/25/2025.
//

#include "../../include/subsystems/localization.hpp"
#include "../../include/ports.hpp"
#include "../../include/pros/imu.hpp"
#include "../../include/pros/imu.h"
#include "../../include/controller.hpp"
#include "../../include/lemlib/pose.hpp"
#include "../../include/subsystems/drivetrain.hpp"
#include "../../include/units/units.hpp"
#include "../../include/cls/localization_utils.hpp"
#include "../../include/locolib/config.hpp"
#include "../../include/eigen/Eigen"
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


/*
 * For offsets,
 * X axis is front to back
 * Y axis is side to side
 */
namespace loc_offsets
{
    const vector front(5.75,5.75,0);
    constexpr vector::axis front_axis = vector::axis::Y;

    const vector left(5,4.25,90);
    constexpr vector::axis left_axis = vector::axis::X;

    const vector rear(5.75,5,180);
    constexpr vector::axis rear_axis = vector::axis::Y;

    const vector right(5,4.25,270);
    constexpr vector::axis right_axis = vector::axis::X;
}

/// timepoint value representing the last time the tick function was ran and updated this variable
static std::uint32_t time_at_last_call;

localization::localization() :
        inertial(INERTIAL),
        rotation_vertical(ROTATION_VERTICAL),
        tracking_vertical(&rotation_vertical, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_HORIZONTAL),
        odom_sensors(&tracking_vertical, nullptr, nullptr, nullptr, &inertial),
        estimated_velocity(0,0,0),
        estimated_position(0,0,0),
        rear_loc(loc_offsets::rear, loc_offsets::front_axis, REAR_LOC),
        right_loc(loc_offsets::right, loc_offsets::right_axis, LEFT_LOC),
        left_loc(loc_offsets::left, loc_offsets::left_axis, RIGHT_LOC),
        front_loc(loc_offsets::front, loc_offsets::front_axis, FRONT_LOC),
        monte_task(nullptr)
{
    time_at_last_call = pros::millis();
}

void localization::tick_implementation()
{
    // Acceleration vector. Acquire before calculations.
    pros::imu_raw_s accel_raw = inertial.get_gyro_rate();
    pros::imu_raw_s accel = pros::imu_raw_s();
    accel.x = accel_raw.x * 0.980665f;
    accel.y = accel_raw.y * 0.980665f;
    accel.x = 0;

    auto now = pros::millis();
    auto duration = now - time_at_last_call;
    long long duration_s = duration / 1000;
    time_at_last_call = now;

    //Vf = Vo + A * T
    estimated_velocity.x = estimated_velocity.x + accel.x * duration_s;
    estimated_velocity.y = estimated_velocity.y + accel.y * duration_s;
    estimated_velocity.z = estimated_velocity.z + accel.z * duration_s;

    // Delta X = Vo * T + 1/2 * A * T^2
    // X = X + Delta X
    estimated_position.x += estimated_velocity.x * duration_s + 0.5 * accel.x * (duration_s * duration_s);
    estimated_position.y += estimated_velocity.y * duration_s + 0.5 * accel.y * (duration_s * duration_s);
    estimated_position.z += estimated_velocity.z * duration_s + 0.5 * accel.z * (duration_s * duration_s);
}

localization* localization::get()
{
    if (!odometry_instance) odometry_instance = std::unique_ptr<localization>(new localization() );
    return odometry_instance.get();
}

void localization::distance_sensor_reset(localization_update update_type)
{
    float front_dist = front_loc.distance_raw();
    float rear_dist = rear_loc.distance_raw();
    float right_dist = right_loc.distance_raw();
    float left_dist = left_loc.distance_raw();

    lemlib::Pose curPose = drivetrain::get()->lem_chassis.getPose();

    //In degrees
    float heading = curPose.theta;

    switch (update_type) {

        case SKILLS_INITIAL:
        {
            rear_dist = -wall_coord + rear_dist;
            left_dist = -wall_coord + left_dist;
            drivetrain::get()->lem_chassis.setPose(left_dist, rear_dist, heading);
            break;
        }

        case AUTON_INITIAL_LEFT:
        {
            front_dist = -wall_coord + front_dist;
            right_dist = wall_coord - right_dist;
            drivetrain::get()->lem_chassis.setPose(front_dist, right_dist, heading);
            break;
        }

        case AUTON_INITIAL_RIGHT:
        {
            front_dist = -wall_coord + front_dist;
            left_dist = -wall_coord + left_dist;
            drivetrain::get()->lem_chassis.setPose(front_dist, left_dist, heading);
            break;
        }

        case MATCH_LOADER_1:
        {
            rear_dist = wall_coord - rear_dist;
            right_dist = wall_coord - right_dist;
            drivetrain::get()->lem_chassis.setPose(left_dist, rear_dist, heading);
            break;
        }

        case MATCH_LOADER_2:
        {
            rear_dist = -wall_coord + rear_dist;
            left_dist = wall_coord - left_dist;
            drivetrain::get()->lem_chassis.setPose(rear_dist, left_dist, heading);
            break;
        }

        case MATCH_LOADER_3:
        {
            if (heading != 90)
            {
                //Angle of error in radians
                float angle = abs(90-heading) * (M_PI / 180);
                rear_dist = cos(angle) * rear_dist;
                right_dist = cos(angle) * right_dist;
            }

            rear_dist = -wall_coord + rear_dist;
            right_dist = -wall_coord + right_dist;
            drivetrain::get()->lem_chassis.setPose(rear_dist, right_dist, heading);
            break;
        }

        case MATCH_LOADER_4:
        {
            rear_dist = wall_coord - rear_dist;
            left_dist = -wall_coord + left_dist;
            drivetrain::get()->lem_chassis.setPose(rear_dist, left_dist, heading);
            break;
        }
    }
}

void localization::do_localization(lemlib::Chassis* chassis)
{
    return ;
}

void localization::start_localization()
{
    if(monte_task != nullptr)
    {
        stop_localization();
        return;
    }

    distance_sensor_reset(SKILLS_INITIAL);

    monte_task = new pros::Task([this]() -> void
    {
        lemlib::Chassis* chassis = &drivetrain::get()->lem_chassis;

        while (true) {
            this->do_localization(chassis);
            pros::Task::delay(20);
        }
    });
}

void localization::stop_localization()
{
    if(monte_task == nullptr) return;
    monte_task->suspend();
    delete monte_task;
    monte_task = nullptr;
}
