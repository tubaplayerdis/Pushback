//
// Created by aaron on 11/21/2025.
//

#ifndef PUSHBACK_LOCALIZATION_UTILS_HPP
#define PUSHBACK_LOCALIZATION_UTILS_HPP

#include "../pros/distance.hpp"
#include "../locolib/distance.hpp"
#include "../locolib/particleFilter.hpp"
#include "../locolib/config.hpp"
#include <optional>

/*
 * 3D/2D data structure.
 * When used in localization, Z is representative of the theta (angle) of the object this refers to.
 */
struct vector
{
    /*
     * Enum representing the axis of vector. Useful when describing an important axis of the vector
     */
    enum axis
    {
        X,
        Y,
        Z
    };

    float x;
    float y;
    float z;

    vector()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    vector(float X, float Y, float Z)
    {
        x = X;
        y = Y;
        z = Z;
    }
};

/*
 *
 */
struct localization_sensor
{
    static constexpr int err_reading_value = 9999;
    static constexpr float mm_inch_conversion_factor = 0.0393701;

    /// Offset of the distance sensor in inches from the center of the robot on the axis of sight direction. So if the sensor looked in the Y direction, the offset would be the offset in the Y direction and not the X direction.
    const vector offset;
    const vector::axis localization_axis;

    /// Pros distance sensor object
    pros::Distance sensor;

    //Add sensor model as struct member and change accessor to return pointer to model
    loco::DistanceSensorModel sensor_model;

    /// Constructor
    localization_sensor(vector off, vector::axis axis, int port) :
            offset(off),
            localization_axis(axis),
            sensor(port),
            sensor_model({off.x, off.y, off.z}, sensor)
    {

    }

    /// Returns distance with offset added to the sensor reading in INCHES.
    std::optional<float> distance()
    {
        int sensor_reading = sensor.get_distance();
        if (sensor_reading == err_reading_value) return std::nullopt;

        float offset_val = 0;

        switch (localization_axis) {
            case vector::X:
            {
                offset_val = offset.x;
                break;
            }
            case vector::Y:
            {
                offset_val = offset.y;
                break;
            }
            case vector::Z:
            {
                offset_val = offset.z;
                break;
            }
        }

        return (double)sensor_reading * mm_inch_conversion_factor + offset_val;
    }

    loco::DistanceSensorModel* get_sensor_model()
    {
        return &sensor_model;
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
 * Structure used to store the data of the localization system.
 */
struct localization_data
{
    std::ranlux24_base random_gen;
    QLength odom_change;
    QLength last_odom;
    Angle last_theta;
    Eigen::Vector3f exponential_pose;

    localization_data() :
    random_gen(),
    odom_change(0.0),
    last_odom(0.0),
    last_theta(0.0),
    exponential_pose(Eigen::Vector3f::Zero())
    {}
};

#endif //PUSHBACK_LOCALIZATION_UTILS_HPP
