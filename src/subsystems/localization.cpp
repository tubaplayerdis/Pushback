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
#include <memory>
#include <chrono>
#include <cstring>
#include <optional>
#include <random>


std::unique_ptr<localization> odometry_instance;

using namespace ports::localization;
using namespace ports::localization::settings;


/*
 * For offsets,
 * X axis is front to back
 * Y axis is side to side
 */
namespace loc_offsets
{
    const vector front(0,0,0);
    constexpr vector::axis front_axis = vector::axis::X;

    const vector left(0,0,90);
    constexpr vector::axis left_axis = vector::axis::Y;

    const vector rear(0,0,180);
    constexpr vector::axis rear_axis = vector::axis::X;

    const vector right(0,0,270);
    constexpr vector::axis right_axis = vector::axis::Y;
}

localization::localization() :
        inertial(INERTIAL),
        rotation_vertical(ROTATION_VERTICAL),
        tracking_vertical(&rotation_vertical, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_HORIZONTAL),
        odom_sensors(&tracking_vertical, nullptr, nullptr, nullptr, &inertial),
        rear_loc(loc_offsets::rear, loc_offsets::front_axis, REAR_LOC),
        right_loc(loc_offsets::right, loc_offsets::right_axis, LEFT_LOC),
        left_loc(loc_offsets::left, loc_offsets::left_axis, RIGHT_LOC),
        front_loc(loc_offsets::front, loc_offsets::front_axis, FRONT_LOC),

        //Function that allows the particle filter to interpret updates to the angle (heading) of the robot. passed as a lambda
        particle_filter([this]() {
            const Angle angle = -inertial.get_rotation() * degree;
            return isfinite(angle.getValue()) ? angle : 0.0;
        }),
        last_odom(0.0),
        odom_change(0.0),
        last_theta(0.0),
        exponential_pose(Eigen::Vector3f::Zero());
        monte_task(nullptr)
{
    //Add particle filter sensors
    particle_filter.addSensor(front_loc.get_sensor_model());
    particle_filter.addSensor(right_loc.get_sensor_model());
    particle_filter.addSensor(rear_loc.get_sensor_model());
    particle_filter.addSensor(left_loc.get_sensor_model());
    last_odom = get_odom_distance();
}

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
    } catch (std::bad_variant_access& err)
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

QLength localization::get_odom_distance()
{
    auto distance = tracking_vertical.get_offset();
    return distance;
}

void localization::do_localization()
{
    const QLength odomReading = get_odom_distance();

    odom_change = odomReading - last_odom;

    last_odom = odomReading;

    std::uniform_real_distribution<> avgDistribution(odom_change.getValue() - loco::LOCO_CONFIG::DRIVE_NOISE * odom_change.getValue(),
                                                   odom_change.getValue() + loco::LOCO_CONFIG::DRIVE_NOISE * odom_change.getValue());
    std::uniform_real_distribution<> angleDistribution(
            particle_filter.getAngle().getValue() - loco::LOCO_CONFIG::ANGLE_NOISE.getValue(),
            particle_filter.getAngle().getValue() + loco::LOCO_CONFIG::ANGLE_NOISE.getValue());

    // Exponential Pose Tracking
    const Angle dTheta = particle_filter.getAngle() - last_theta;

    const auto localMeasurement = Eigen::Vector2f({odom_change.getValue(), 0});
    const auto displacementMatrix =
            Eigen::Matrix2d({
                                    {1.0 - pow(dTheta.getValue(), 2), -dTheta.getValue() / 2.0},
                                    {dTheta.getValue() / 2.0, 1.0 - pow(dTheta.getValue(), 2)}
                            })
                    .cast<float>();

    auto time = pros::micros();

    particle_filter.update([this, angleDistribution, avgDistribution, displacementMatrix]() mutable {
        const auto noisy = avgDistribution(de);
        const auto angle = angleDistribution(de);

        return Eigen::Rotation2Df(angle) * Eigen::Vector2f({noisy, 0.0});
    });

    const Eigen::Vector2f localDisplacement = displacementMatrix * localMeasurement;
    const Eigen::Vector2f globalDisplacement =
            Eigen::Rotation2Df(particle_filter.getAngle().Convert(radian)) * localDisplacement;

    exponential_pose += Eigen::Vector3f(globalDisplacement.x(), globalDisplacement.y(), dTheta.Convert(radian));

    last_theta = particle_filter.getAngle();
}
