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
    static constexpr int err_reading_value = 9999;
    static constexpr float mm_inch_conversion_factor = 0.0393701;

    /// Offset of the distance sensor in inches from the center of the robot on the axis of sight direction. So if the sensor looked in the Y direction, the offset would be the offset in the Y direction and not the X direction.
    const float offset;

    /// Pros distance sensor object
    pros::Distance sensor;

    /// Constructor
    localization_sensor(float off, int port) : offset(off), sensor(port) {}

    /// Returns distance with offset added to the sensor reading in INCHES.
    std::optional<float> distance()
    {
        int sensor_reading = sensor.get_distance();
        if (sensor_reading == err_reading_value) return std::nullopt;
        return (double)sensor_reading * mm_inch_conversion_factor + offset;
    }

    [[nodiscard]] loco::DistanceSensorModel as_sensor_model(float x_off, float y_off, float h_off) const
    {
        return {{x_off, y_off, h_off}, sensor};
    }
};

/// Passed the distance sensor reset to inform it how to process data.
/// All localization locations are based in quadrants which are defined like you are looking at 4 quadrants on the field from the driver position. They are as follows:
/// Top Right (+,+)
/// Top Left (-,+)
/// Bottom Left (-,-)
/// Bottom Right (+,-)
enum localization_update
{
    /// Initial starting location of skills, 90 degrees turned with the match loader facing the wall to the right
    SKILLS_INITIAL,

    /// Initial starting location of left side autons with the aligner facing towards the wall
    AUTON_INITIAL_LEFT,

    /// Initial starting location of right side autons with the aligner facing towards the wall
    AUTON_INITIAL_RIGHT,

    /// Match loader top left of driver location. ++ quadrant
    MATCH_LOADER_1,

    /// Match loader bottom left of driver location. -+ quadrant
    MATCH_LOADER_2,

    /// Match loader bottom right of driver location. -- quadrant
    MATCH_LOADER_3,

    /// Match loader bottom left of driver location, closest to driver. +- quadrant
    MATCH_LOADER_4,
};

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

    /// Pros task that handles monte carlo localization.
    pros::Task* monte_task;

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
