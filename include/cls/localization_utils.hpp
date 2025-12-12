//
// Created by aaron on 11/21/2025.
//

// Utilities used by localization subsystem for distance sensor resets and monte carlo localization.

#ifndef PUSHBACK_LOCALIZATION_UTILS_HPP
#define PUSHBACK_LOCALIZATION_UTILS_HPP

#include "../pros/distance.hpp"
#include "../pros/rtos.hpp"
#include <optional>

#include "../lemlib/chassis/chassis.hpp"
#include "../pros/imu.hpp"

/**
 * Standard probability type definition
 */
typedef float t_probability;

/**
 * Confidence Pair
 *
 * Templated pair abstraction with the confidence value as an unsigned char for memory usage optimization
 */
template<typename T>
class conf_pair
{
private:
    /**
     * Internal pair value
     */
    std::pair<T, t_probability>value;

public:

    /**
     * @brief confidence pair default constructor
     */
    conf_pair()
    {
        value = std::pair<T, t_probability>(T(), 0);
    }

    /**
     * @brief confidence pair passing the confidence as an unsigned char
     *
     * @param principal value of the pair
     * @param confidence confidence as a unsigned char
     */
    conf_pair(T principal, t_probability confidence)
    {
        value = std::pair<T, t_probability>(principal, confidence);
    }

    /**
     * @brief Sets the confidence of the confidence pair
     * @param confidence new confidence to set
     */
    void set_confidence(t_probability confidence)
    {
        value.second = confidence;
    }

    /**
     * @breif Sets the value of the confidence pair
     * @param principal new value to set
     */
    void set_value(T principal)
    {
        value.first = principal;
    }


    /**
     * @brief Gets the value of the confidence pair as T
     * @return first value of the internal pair as T
     */
    T get_value()
    {
        return value.first;
    }

    /**
     * @brief Gets the confidence of the confidence pair as a float
     * @return confidence of the internal pair as a float
     */
    t_probability get_confidence()
    {
        return value.second;
    }
};

/**
 * Standard distance confidence pair definition
 */
typedef conf_pair<float> t_distance;

/**
 * Standard particle confidence pair definition.
 */
typedef conf_pair<std::pair<float, float>> t_particle;


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

struct localization_options
{
    /**
     * Trust in the distance sensors around the robot.
     */
    const unsigned char sensor_trust = 155;

    /**
     * Trust in the odometry system of the robot.
     */
    const unsigned char odometry_trust = 240;

    /**
     * Percentage at which the sensor values correct odometry as to smooth out correction
     */
    const float sensor_correction_gain = 0.25;
};

/**
 * @brief Distance sensor wrapper class used for distance sensor resets and localization.
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
     * @brief Constructor for localization sensor.
     * @note Offsets should be done in inches.
     * @param off Offset of the sensor from the origin in the direction the sensor is facing
     * @param port Port of the distance sensor.
     */
    localization_sensor(float off, int port);

    /**
     * @brief Distance read from the distance sensor as a confidence pair.
     * @note Data returned is in inches.
     * @return confidence and the distance reading.
     */
    t_distance distance();

    /**
     * @brief Distance read from the distance sensor as a confidence pair with angle of robot factored in and offset.
     * @note Data returned is in inches.
     * @return confidence and the distance reading and calculation.
     */
    t_distance distance(float heading);
};

class localization_chassis
{
    /**
     * @brief Normalizes heading to the domain of 0-360.
     * @param heading heading to normalize.
     * @return Normalized heading
     */
    float normalize_heading(float heading);

    /**
     * @brief Returns the relevant sensors based on the heading of the robot.
     *
     * Requires normalized heading 0-360 degrees.
     *
     * For 315 - 45 degrees: ++
     * For 45 - 135 degrees: -+
     * For 135 - 225 degrees: --
     * For 225 - 315 degrees: +-
     */
    quadrant sensor_relevancy();

    /**
     * @brief Average confidence of value pair
     * @param one first confidence pair
     * @param two second confidence pair
     * @return Average confidence of value pair
     */
    static float conf_avg(conf_pair<float> one, conf_pair<float> two);

    /**
     * @brief Returns the confidence pair of a coordinate pair representing the robots location gathered from the sensors.
     * @param quad Current quadrant of the robot
     * @return Confidence pair of a coordinate pair representing the robots location gathered from the sensors.
     */
    conf_pair<std::pair<float, float>> get_position_calculation(quadrant quad);

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

    /**
     * Last call time of filtering
     */
    int32_t last_call_time;

    /**
     * Provided options
     */
    localization_options options;

public:

    /**
     * Initialize the localization chassis
     */
    localization_chassis(localization_options settings, pros::Imu* inertial, lemlib::Chassis* base, std::array<localization_sensor*,4> sensors);

    /**
     * @brief Performs a distance sensor reset using the sensors on the robot given the robot already knows where it is.
     *
     * Uses the LemLib chassis position along with the probability readings from the sensors to determine if the
     * location generated from the sensors is accurate. If the readings are determined to be accurate, the function will
     * return true and set the location of the LemLib chassis to that of the readings specified, with the function just
     * returning false otherwise and not setting the LemLib chassis's location. Do not use while the robot is in motion as
     * it will cause jerky and or unpredictable movements. Use when stationary and the robot is in an adequate place to gather readings.
     */
    bool reset_location();

    /**
     * @brief Performs a distance sensor reset using the sensors on the robot given the robot does not know where it is and the sensors are fully trusted.
     *
     * @param quad The quadrant the robot is currently in.
     * @return Whether the function was successfully in reset the location of the robot.
     */
    bool reset_location_force(quadrant quad);
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
