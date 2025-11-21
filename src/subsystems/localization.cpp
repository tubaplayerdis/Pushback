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
        right_loc(LEFT, LEFT_LOC),
        left_loc(RIGHT, RIGHT_LOC),
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
    //Since the field is 12ft x 12ft, each quadrant is 72in x 72in. This is used as a global offset to the distance sensor readings to accurately map where the robot is.
    static constexpr double wall_coord = 72;

    double front_dist = 0;
    double rear_dist = 0;
    double right_dist = 0;
    double left_dist = 0;

    try
    {
        front_dist = front_loc.distance().value();
        rear_dist = rear_loc.distance().value();
        right_dist = right_loc.distance().value();
        left_dist = left_loc.distance().value();
    } catch (std::bad_optional_access& err)
    {
        std::printf("Could not perform a distance sensor reset!");
        controller_master.rumble(".-.-.");
        pros::Task::delay(100);
        return;
    }

    lemlib::Pose curPose = drivetrain::get()->lem_chassis.getPose();
    double heading = curPose.theta;

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
