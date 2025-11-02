//
// Created by aaron on 8/25/2025.
//
#include "../../include/subsystems/localization.hpp"
#include "../../include/ports.hpp"
#include "../../include/pros/imu.hpp"
#include "../../include/pros/imu.h"
#include "../../include/controller.hpp"
#include <memory>
#include <chrono>
#include <cstring>


std::unique_ptr<localization> odometry_instance;

using namespace ports::localization;
using namespace ports::localization::settings;

/// timepoint value representing the last time the tick function was ran and updated this variable
static std::chrono::time_point<std::chrono::high_resolution_clock> time_at_last_call;

localization::localization() :
inertial(INERTIAL),
rotation_vertical(ROTATION_VERTICAL),
game_positioning_system_sensor(GPS),
tracking_vertical(&rotation_vertical, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_HORIZONTAL),
odom_sensors(&tracking_vertical, nullptr, nullptr, nullptr, &inertial),
estimated_velocity(0,0,0),
estimated_position(0,0,0)
{
    time_at_last_call = std::chrono::high_resolution_clock::now();
}

void localization::tick_implementation() {
    /*
     * Position estimation system.
     * This system is an experiment on whether the inertial sensor can provide accurate values to estimate position and velocity.
     * The system is based on kinematics fundamentals which states that position can be found given time and acceleration, both of which can be retrieved from either the robot or the inertial sensor.
     * The accuracy of this system is untested but the last time I implemented it I got erroneous values due to the time estimation being wrong.
     */

    //Return early for now.
    return;

    // Acceleration vector. Acquire before calculations.
    pros::imu_raw_s accel = inertial.get_accel();

    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now - time_at_last_call;
    long long duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    long long duration_s = duration_ms / 1000;
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

vector localization::get_estimated_velocity() {
    //Manual std::memcpy for performance. unimportant to the return value.
    vector ret;
    std::memcpy(&ret, &estimated_velocity, sizeof(vector));
    return ret;
}

vector localization::get_estimated_position() {
    //Manual std::memcpy for performance. unimportant to the return value.
    vector ret;
    std::memcpy(&ret, &estimated_position, sizeof(vector));
    return ret;
}
