//
// Created by aaron on 11/21/2025.
//

// Utilities used by localization subsystem for distance sensor resets and monte carlo localization.

#ifndef PUSHBACK_LOCALIZATION_UTILS_HPP
#define PUSHBACK_LOCALIZATION_UTILS_HPP

#include "../pros/distance.hpp"
#include "../locolib/distance.hpp"
#include "../locolib/particleFilter.hpp"
#include "../locolib/config.hpp"
#include "../pros/rtos.hpp"
#include <optional>

/*
 * 3D/2D data structure.
 * When used in localization, Z is representative of the theta (angle) of the object this refers to.
 */
struct vector
{
    /**
     * @brief Enum representing the axes of vector. Useful when describing an important axis of the vector
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

    /**
     * @breif Default constructor for vector, initializes X, Y, and Z to zero.
     */
    vector();

    /**
     * @brief Standard constructor for vector, initializes values to input parameters
     * @param X x value of vector
     * @param Y y value of vector
     * @param Z z or theta value of vector depending on plane interpreted
     */
    vector(float X, float Y, float Z);
};

/**
 * @brief Distance sensor wrapper class used for distance sensor resets and monte carlo localization.
 */
class localization_sensor
{
    /**
     * Offset vector of the localization sensor.
     * X is front to back, Y is side to side, Z is theta of sensor.
     */
    const vector offset;

    /**
     * Important axis of the localization sensor to be used when calculating distance.
     */
    const vector::axis localization_axis;

    /**
     * Pros distance sensor object.
     */
    pros::Distance sensor;

    /**
     * Locolib distance sensor model.
     */
    loco::DistanceSensorModel sensor_model;

    public:

    /**
     * @brief Constructor for localization sensor. Vector interpretation is 2D with the X axis being front to back, Y axis side to side, and Z the theta.
     * @note Offsets should be done in inches.
     * @param off Offset of the sensor on a 2D plane with Z representing the theta of the sensor.
     * @param axis Important axis of the sensor, the direction it senses in. Used in distance sensor resets.
     * @param port Port of the distance sensor.
     */
    localization_sensor(vector off, vector::axis axis, int port);

    /**
     * @brief Distance read from the distance sensor along with the important axis's offset added.
     * @note Data returned is in inches.
     * @return nullopt or the distance reading and calculation.
     */
    std::optional<float> distance();

    /**
     * Retrieve the locolib distance sensor model pointer created by the localization sensor.
     * @return
     */
    loco::DistanceSensorModel* get_sensor_model();
};

/**
 * @brief Enum used when performing a distance sensor reset. Defines predefined locations that can be used for distance sensor resets.
 * All localization locations are based in quadrants which are defined like you are looking at 4 quadrants on the field from the driver position. They are as follows:
 * Top Right (+,+)
 * Top Left (-,+)
 * Bottom Left (-,-)
 * Bottom Right (+,-)
 */
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

/**
 * @brief Structure used to store the data of the localization system. Used by the monte carlo localization system
 */
struct localization_data
{
    std::ranlux24_base random_gen;
    QLength odom_change;
    QLength last_odom;
    Angle last_theta;
    Eigen::Vector3f exponential_pose;

    localization_data();
};

#endif //PUSHBACK_LOCALIZATION_UTILS_HPP
