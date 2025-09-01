//
// Created by aaron on 7/23/2025.
//

#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "../cls/subsystem.h"
#include "../lemlib/chassis/chassis.hpp"

struct vector
{
    double x;
    double y;
    double z;

    vector(double X, double Y, double Z)
    {
        x = X;
        y = Y;
        z = Z;
    }
};

class odometry final : public subsystem
{
    /// Friend class to allow unique_ptr to access deconstructor
    friend class std::unique_ptr<odometry>;

public:

    /// Inertial sensor responsible for things like velocity and rotation
    pros::Imu inertial;

    /// Pros rotation sensor for vertical wheel.
    pros::Rotation rotation_vertical;

    /// Pros rotation sensor for horizontal wheel.
    pros::Rotation rotation_horizontal;

    /// LemLib horizontal tracking wheel for autons
    lemlib::TrackingWheel tracking_horizontal;

    /// LemLib vertical tracking wheel for autons
    lemlib::TrackingWheel tracking_vertical;

    /// LemLib "odometry" object for autons
    lemlib::OdomSensors odom_sensors;

private:

    /// velocity based off inertial acceleration readings.
    vector estimated_velocity;

    /// position based off inertial acceleration readings.
    vector estimated_position;

    /// Private constructor to enforce usage of get()
    odometry();

protected:

    /// Custom tick implementation for odometry. updates estimated velocity and position values.
    void tick_implementation() override;

public:

    /// public accessor method for singleton.
    static odometry* get();
};

#endif //ODOMETRY_H
