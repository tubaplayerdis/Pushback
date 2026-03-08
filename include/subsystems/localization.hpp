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
#include "../pros/motor_group.hpp"
#include "../lemlib/chassis/chassis.hpp"
#include "../lemlib/chassis/trackingWheel.hpp"
#include "../pros/imu.hpp"
#include "../TitanReset/TitanReset.hpp"
#include "../pros/adi.hpp"

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

    /// Left motor group. Includes 3 motors, A, B, C
    pros::MotorGroup motors_left;

    /// Right motor group. Includes 3 motors, A, B, C
    pros::MotorGroup motors_right;

    /// Inertial sensor responsible for things like velocity and rotation
    pros::Imu inertial;

    /// Pros rotation sensor for vertical wheel
    pros::Rotation rotation_vertical;

    /// Pros encoder for horizontal wheel
    pros::adi::Encoder encoder_horizontal;

    /// LemLib vertical tracking wheel wrapper object
    lemlib::TrackingWheel tracking_vertical;

    /// LemLib horizontal tracking wheel wrapper object
    lemlib::TrackingWheel tracking_horizontal;

    /// LemLib "odometry" object aggregating odometry objects
    lemlib::OdomSensors odom_sensors;

    /// LemLib drivetrain object. Organizes the motors.
    lemlib::Drivetrain lem_drivetrain;

    /// LemLib chassis object. Compiles the Odometry and Drivetrain objects for advanced autonomous creation
    lemlib::Chassis lem_chassis;

    /// Front TitanReset sensor. Located near aligner
    tr_sensor front_loc;

    /// Rear TitanReset sensor. Located near match loader mount.
    tr_sensor rear_loc;

    /// Right TitanReset sensor. Located on the side of the brain.
    tr_sensor right_loc;

    /// Left TitanReset sensor. Located on the opposite side of the brain.
    tr_sensor left_loc;

    /// TitanReset chassis object
    tr_chassis l_chassis;

private:

    /// Private constructor to enforce usage of get()
    localization();

protected:

    /// Custom tick implementation for localization. updates estimated velocity and position values.
    void tick_implementation() override;

public:

    /// public accessor method for singleton.
    static localization* get();
};

#endif //LOCALIZATION_H
