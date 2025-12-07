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

#include "../lemlib/chassis/chassis.hpp"
#include "../pros/imu.hpp"

/*
 * Single axis probability type definition. Pair of float (the value) and unsigned char (the probability 0-255)
 */
typedef std::pair<float, unsigned char> probability;

/*
 * Quadrant enumeration.
 */
enum quadrant
{
    POS_POS,
    NEG_POS,
    NEG_NEG,
    POS_NEG,
};

enum sensor_options
{
    NORTH = 1 << 0,
    EAST = 1 << 2,
    SOUTH = 1 << 3,
    WEST = 1 << 4,
};

/*
 * 3D/2D data structure.
 * When used in localization, Z is representative of the theta (angle) of the object this refers to.
 */
struct vector
{
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
    const float offset;

    /**
     * Pros distance sensor object.
     */
    pros::Distance sensor;

    public:

    /**
     * @brief Constructor for localization sensor. Vector interpretation is 2D with the X axis being front to back, Y axis side to side, and Z the theta.
     * @note Offsets should be done in inches.
     * @param off Offset of the sensor on a 2D plane with Z representing the theta of the sensor.
     * @param port Port of the distance sensor.
     */
    localization_sensor(float off, int port);

    /**
     * @brief Distance read from the distance sensor as an optional.
     * @note Data returned is in inches.
     * @return confidence and the distance reading.
     */
    probability distance();

    /**
     * @brief Distance read from the distance sensor as an optional with angle of robot factored in and offset.
     * @note Data returned is in inches.
     * @return confidence and the distance reading and calculation.
     */
    probability distance(float heading);
};

class localization_chassis
{

    /*
     * Sensors
     */
    localization_sensor* north;
    localization_sensor* east;
    localization_sensor* south;
    localization_sensor* west;
    pros::Imu* imu;

    /*
     * LemLib reference
     */
    lemlib::Chassis* chassis;


    int32_t last_call_time;

public:

    /**
     * Initialize the localization chassis
     */
    localization_chassis(pros::Imu* inertial, lemlib::Chassis* base, std::array<localization_sensor*,4> sensors);

    /**
     * @brief Performs a distance sensor reset using the sensors specified.
     *
     * Uses the LemLib chassis position along with the probability readings from the sensors to determine if the
     * location generated from the sensors is accurate. If the reading is determined to be accurate, the function will
     * return true and set the location of the LemLib chassis to that of the readings specified, with the function just
     * returning false otherwise and not setting the LemLib chassis's location. Do not use while the robot is in motion as
     * it will cause jerky and or unpredictable movements. Use when stationary and the robot is in an adequate place to gather readings.
     *
     *
     * @param ignore_probability ignores the probability statistic. Use when the robot does not know its location (ex: at starting location)
     * @param sensors flags specifying which sensors to use in the distance sensor reset.
     */
    bool reset_location(bool ignore_probability, int sensors);
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

#endif //PUSHBACK_LOCALIZATION_UTILS_HPP
