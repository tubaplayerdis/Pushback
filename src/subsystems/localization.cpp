//
// Created by aaron on 8/25/2025.
//

#include "../../include/subsystems/localization.hpp"
#include "../../include/ports.hpp"
#include "../../include/pros/imu.hpp"
#include "../../include/pros/imu.h"
#include "../../include/controller.hpp"
#include "../../include/lemlib/pose.hpp"
#include <memory>
#include <chrono>
#include <cstring>

#include "../../include/subsystems/drivetrain.hpp"


std::unique_ptr<localization> odometry_instance;

using namespace ports::localization;
using namespace ports::localization::settings;

/// timepoint value representing the last time the tick function was ran and updated this variable
static std::chrono::time_point<std::chrono::high_resolution_clock> time_at_last_call;

localization::localization() :
inertial(INERTIAL),
rotation_vertical(ROTATION_VERTICAL),
tracking_vertical(&rotation_vertical, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_HORIZONTAL),
odom_sensors(&tracking_vertical, nullptr, nullptr, nullptr, &inertial),
estimated_velocity(0,0,0),
estimated_position(0,0,0),
rear_loc(9, REAR),
left_loc(5.5, LEFT),
right_loc(5.5, RIGHT)
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
    /*
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
    */
}

bool w_range(double val, double other, double range)
{
    return val <= other + range && val >= other - range;
}

localization* localization::get()
{
    if (!odometry_instance) odometry_instance = std::unique_ptr<localization>(new localization() );
    return odometry_instance.get();
}

vector localization::get_estimated_velocity() {
    return estimated_velocity;
}

vector localization::get_estimated_position() {
    return estimated_position;
}

void localization::distance_sensor_reset()
{
    const double err_reading = 9999;

    double rear_dist = rear_loc.distance.get_distance() * 0.0393701 + rear_loc.offset;
    double left_dist = left_loc.distance.get_distance() * 0.0393701 + left_loc.offset;
    double right_dist = right_loc.distance.get_distance() * 0.0393701 + right_loc.offset;

    lemlib::Pose curPose = drivetrain::get()->lem_chassis.getPose();
    double heading = curPose.theta;

    // Robot facing intake towards field is 0 degrees

    if (w_range(heading, 180, 3))
    {
        //Robot is against wall near driver
        if (left_dist == err_reading)
        {
            right_dist = 72 - right_dist;
            drivetrain::get()->lem_chassis.setPose(right_dist, rear_dist, heading);
            return;
        }

        if (rear_dist == err_reading)
        {
            left_dist = -72 + left_dist;
            drivetrain::get()->lem_chassis.setPose(left_dist, rear_dist, heading);
            return;
        }

    } else if (w_range(heading, 90, 3))
    {
        //Robot is in the starting position
        rear_dist = -72 + rear_dist;
        right_dist = -72 + right_dist;
        drivetrain::get()->lem_chassis.setPose(rear_dist, right_dist, heading);

    } else if (w_range(heading, 0, 3))
    {
        //Robot is against wall on other side of driver
    }


}
