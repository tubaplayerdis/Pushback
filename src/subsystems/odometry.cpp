//
// Created by aaron on 8/25/2025.
//
#include "../../include/subsystems/odometry.h"
#include "../../include/ports.h"
#include "../../include/pros/imu.hpp"
#include "../../include/pros/imu.h"
#include <memory>
#include <chrono>
#include <cstring>


std::unique_ptr<odometry> odometry_instance;

using namespace ports::odometry;
using namespace ports::odometry::settings;

static std::chrono::time_point<std::chrono::high_resolution_clock> time_at_last_call;

odometry::odometry() :
inertial(INERTIAL),
rotation_vertical(ROTATION_VERTICAL),
rotation_horizontal(ROTATION_HORIZONTAL),
tracking_horizontal(&rotation_horizontal, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_HORIZONTAL),
tracking_vertical(&rotation_vertical, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_VERTICAL),
odom_sensors(&tracking_vertical, nullptr, &tracking_horizontal, nullptr, &inertial),
estimated_velocity(0,0,0),
estimated_position(0,0,0)
{
    inertial.reset(true);
    time_at_last_call = std::chrono::high_resolution_clock::now();
}

void odometry::tick_implementation() {

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

odometry* odometry::get()
{
    if (!odometry_instance) odometry_instance = std::unique_ptr<odometry>( new odometry() );
    return odometry_instance.get();
}

vector odometry::get_estimated_velocity() {
    vector ret;
    std::memcpy(&ret, &estimated_velocity, sizeof(vector));
    return ret;
}

vector odometry::get_estimated_position() {
    vector ret;
    std::memcpy(&ret, &estimated_position, sizeof(vector));
    return ret;
}
