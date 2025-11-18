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

/// 3 dimensional vector structure
struct vector
{
    double x;
    double y;
    double z;

    vector()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    vector(double X, double Y, double Z)
    {
        x = X;
        y = Y;
        z = Z;
    }
};

struct localization_sensor
{
    double offset;
    pros::Distance distance;

    localization_sensor(double off, int port) : offset(off), distance(port) {}
};

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

    /// LemLib "localization" object for autons
    lemlib::OdomSensors odom_sensors;

    /// Rear localization sensor
    localization_sensor rear_loc;

    /// Left localization sensor
    localization_sensor left_loc;

    /// Right localization sensor
    localization_sensor right_loc;

private:

    /// Velocity based off inertial sensor acceleration readings.
    vector estimated_velocity;

    /// Position based off inertial sensor acceleration readings.
    vector estimated_position;

    /// Private constructor to enforce usage of get()
    localization();

protected:

    /// Custom tick implementation for localization. updates estimated velocity and position values.
    void tick_implementation() override;

public:

    /// Accessor for estimated velocity that keeps passes by pure value copy
    vector get_estimated_velocity();

    /// Accessor for estimated position that keeps passes by pure value copy
    vector get_estimated_position();

    void distance_sensor_reset();

    /// public accessor method for singleton.
    static localization* get();
};

#endif //LOCALIZATION_H
