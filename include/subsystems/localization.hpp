//
// Created by aaron on 7/23/2025.
//

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <memory>
#include "../cls/subsystem.hpp"
#include "../pros/rotation.hpp"
#include "../pros/distance.hpp"
#include "../pros/gps.hpp"
#include "../lemlib/chassis/chassis.hpp"
#include "../locolib/distance.hpp"
#include "../locolib/particleFilter.hpp"
#include "../locolib/config.hpp"
#include "../cls/localization_utils.hpp"

/*
 * For the localization class, the robot's sides are approached unconventionally due to the odometry setup.
 * The front of the robot is side with the exhaust and aligner.
 * The rear/back is the side with the match loader and intake.
 * The left is the non-brain side.
 * The right is the side with the brain.
 */
class localization final : public subsystem
{
    /// Friend class to allow unique_ptr to access deconstructor
    friend class std::unique_ptr<localization>;

public:
    /// Inertial sensor responsible for things like velocity and rotation
    pros::Imu inertial;

    /// Pros rotation sensor for vertical wheel
    pros::Rotation rotation_vertical;

    /// LemLib vertical tracking wheel for autons
    lemlib::TrackingWheel tracking_vertical;

    /// LemLib "odometry" object for autons
    lemlib::OdomSensors odom_sensors;

    /// Front localization sensor. Located near aligner
    localization_sensor front_loc;

    /// Rear localization sensor. Located near match loader mount.
    localization_sensor rear_loc;

    /// Right localization sensor. Located on the side of the brain.
    localization_sensor right_loc;

    /// Left localization sensor. Located on the opposite side of the brain.
    localization_sensor left_loc;

private:

    loco::ParticleFilter<loco::LOCO_CONFIG::NUM_PARTICLES> particle_filter;

    /// Pros task that handles monte carlo localization.
    pros::Task* monte_task;

    QLength odom_change;

    QLength last_odom;

    Angle last_theta;

    Eigen::Vector3f exponential_pose;

    QLength get_odom_distance();

    /// Do not call directly.
    void do_localization();

    /// Private constructor to enforce usage of get()
    localization();

protected:

    /// Custom tick implementation for localization. updates estimated velocity and position values.
    void tick_implementation() override;

public:

    /// Resets the pose of the lemlib chassis object during skills or before autons.
    /// @note Set the heading of the robot beforehand. An example is for skills where the robot needs to be told it is 90 degrees turned.
    void distance_sensor_reset(localization_update update_type);

    /// public accessor method for singleton.
    static localization* get();
};

#endif //LOCALIZATION_H
