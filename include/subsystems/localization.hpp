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
#include "../lemlib/chassis/trackingWheel.hpp"
#include "../pros/imu.hpp"
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

    /// LemLib drivetrain object. Used in autons and the chassis object
    lemlib::Drivetrain lem_drivetrain;

    /// LemLib chassis object. Used in autons and for controller inputs
    lemlib::Chassis lem_chassis;

    /// Front localization sensor. Located near aligner
    localization_sensor front_loc;

    /// Rear localization sensor. Located near match loader mount.
    localization_sensor rear_loc;

    /// Right localization sensor. Located on the side of the brain.
    localization_sensor right_loc;

    /// Left localization sensor. Located on the opposite side of the brain.
    localization_sensor left_loc;

    localization_chassis l_chassis;

public:

    void set_estimated_position(vector3 position)
    {
        estimated_position = position;
    }

    /// Velocity based off inertial sensor acceleration readings.
    vector3 estimated_velocity;

    /// Position based off inertial sensor acceleration readings.
    vector3 estimated_position;

private:

    /// Private constructor to enforce usage of get()
    localization();

protected:

    /// Custom tick implementation for localization. updates estimated velocity and position values.
    void tick_implementation() override;

public:

    /// Resets the pose of the lemlib chassis object during skills or before autons.
    /// @note Set the heading of the robot beforehand. An example is for skills where the robot needs to be told it is 90 degrees turned.
    void distance_sensor_reset();

    /// public accessor method for singleton.
    static localization* get();
};

#endif //LOCALIZATION_H
