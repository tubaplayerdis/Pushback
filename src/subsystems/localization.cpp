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
using namespace ports::localization::offsets;

localization::localization() :
inertial(INERTIAL),
rotation_vertical(ROTATION_VERTICAL),
tracking_vertical(&rotation_vertical, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_HORIZONTAL),
odom_sensors(&tracking_vertical, nullptr, nullptr, nullptr, &inertial),
rear_loc(REAR, REAR_LOC),
left_loc(LEFT, LEFT_LOC),
right_loc(RIGHT, RIGHT_LOC),
front_loc(FRONT, FRONT_LOC)
{}

void localization::tick_implementation()
{
    //If anything is ever used that requires an update on the sensor readings like mcl, implement it here
}

localization* localization::get()
{
    if (!odometry_instance) odometry_instance = std::unique_ptr<localization>(new localization() );
    return odometry_instance.get();
}

void localization::distance_sensor_reset(localization_update update_type)
{
    //Value returned when the distance sensor cannot read a distance
    static constexpr double err_reading = 9999;

    //Since the field is 12ft x 12ft, each quadrant is 72in x 72in. This is used as a global offset to the distance sensor readings to accurately map where the robot is.
    static constexpr double wall_coord = 72;

    double front_dist = front_loc.distance();
    double rear_dist = rear_loc.distance();
    double left_dist = left_loc.distance();
    double right_dist = right_loc.distance();

    lemlib::Pose curPose = drivetrain::get()->lem_chassis.getPose();
    double heading = curPose.theta;

    switch (update_type) {

        case SKILLS_INITIAL:
        {
            rear_dist = -wall_coord + rear_dist;
            right_dist = -wall_coord + right_dist;
            drivetrain::get()->lem_chassis.setPose(right_dist, rear_dist, heading);
            break;
        }

        case AUTON_INITIAL_LEFT:
        {
            front_dist = -wall_coord + front_dist;
            right_dist = wall_coord - right_dist;
            drivetrain::get()->lem_chassis.setPose(right_dist, front_dist, heading);
            break;
        }

        case AUTON_INITIAL_RIGHT:
        {
            front_dist = -wall_coord + front_dist;
            left_dist = -wall_coord + left_dist;
            drivetrain::get()->lem_chassis.setPose(left_dist, front_dist, heading);
            break;
        }

        case MATCH_LOADER_1:
        {
            rear_dist = wall_coord - rear_dist;
            right_dist = wall_coord - right_dist;
            drivetrain::get()->lem_chassis.setPose(right_dist, rear_dist, heading);
            break;
        }

        case MATCH_LOADER_2:
        {
            rear_dist = wall_coord - rear_dist;
            left_dist = -wall_coord + left_dist;
            drivetrain::get()->lem_chassis.setPose(left_dist, rear_dist, heading);
            break;
        }

        case MATCH_LOADER_3:
        {
            rear_dist = -wall_coord + rear_dist;
            right_dist = -wall_coord + right_dist;
            drivetrain::get()->lem_chassis.setPose(right_dist, rear_dist, heading);
            break;
        }

        case MATCH_LOADER_4:
        {
            rear_dist = -wall_coord + rear_dist;
            left_dist = wall_coord - left_dist;
            drivetrain::get()->lem_chassis.setPose(left_dist, rear_dist, heading);
            break;
        }
    }
}
