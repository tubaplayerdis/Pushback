//
// Created by aaron on 11/21/2025.
//

// Utilities used by localization subsystem for distance sensor resets and monte carlo localization.

#ifndef PUSHBACK_LOCALIZATION_UTILS_HPP
#define PUSHBACK_LOCALIZATION_UTILS_HPP

#include "../pros/distance.hpp"
#include "../pros/rtos.hpp"
#include "../pros/imu.h"
#include <optional>
#include <cmath>

#include "../lemlib/chassis/chassis.hpp"
#include "../lemlib/pose.hpp"
#include "../pros/imu.hpp"

/**
 * Standard probability type definition
 */
typedef float t_probability;

/**
 * Confidence Pair
 *
 * Templated pair abstraction with the confidence value.
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
     * @brief confidence pair passing the confidence as a float
     *
     * @param principal value of the pair
     * @param confidence confidence as a float
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
 * 3D vector data structure. Z might be expressed a Theta in some use cases
 */
struct vector3
{
    float x;
    float y;
    float z;

    /**
     * @breif Default constructor for vector, initializes X, Y, and Z to zero.
     */
    vector3()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    /**
     * @brief Standard constructor for vector, initializes values to input parameters
     * @param X x value of vector
     * @param Y y value of vector
     * @param Z z or theta value of vector depending on plane interpreted
     */
    vector3(float X, float Y, float Z)
    {
        x = X;
        y = Y;
        z = Z;
    }
};

struct vector2
{
    float x;
    float y;

    vector2()
    {
        x = 0.0f;
        y = 0.0f;
    }

    vector2(float X, float Y)
    {
        x = X;
        y = Y;
    }
};

struct rectangle
{
    vector2 lower_bound;
    vector2 upper_bound;

    rectangle(const float lx, const float ly, const float ux, const float uy) : lower_bound(lx, ly), upper_bound(ux, uy) {}
    rectangle(vector2 l, vector2 u) : lower_bound(l), upper_bound(u) {}

    bool inside(float x, float y) const
    {
        return inside(vector2(x, y));
    }

    bool inside(vector2 pos) const
    {
        return (pos.x >= lower_bound.x && pos.x <= upper_bound.x) && (pos.y >= lower_bound.y && pos.y <= upper_bound.y);
    }
};

struct circle
{
    vector2 center;
    float radius;

    circle(float x, float y, float r) : center(x, y), radius(r) {}
    circle(vector2 c, float r) : center(c), radius(r) {}

    bool inside(float x, float y)
    {
        return inside(vector2(x, y));
    }

    bool inside(vector2 pos) const
    {
        //distance formula
        return sqrt(std::pow(pos.x - center.x,2) + std::pow(pos.y - center.y, 2)) <= radius;
    }
};

enum sensors
{
    NORTH = 1,
    EAST = 2,
    SOUTH = 4,
    WEST = 8,
};

struct localization_data
{
    /**
     * Time of data point in MS
     */
    int32_t call_time;

    /**
     * Acceleration vector as direction and magnitude
     */
    vector2 last_acceleration;

    /**
     * Velocity vector as direction and magnitude
     */
    vector2 last_velocity;

    /**
     * Last position
     */
    lemlib::Pose last_position;

    localization_data(int32_t ct, vector2 la, vector2 lv, lemlib::Pose lp) : last_position(lp)
    {
        call_time = ct;
        last_acceleration = la;
        last_velocity = lv;
    }
};

struct localization_options
{
    /**
     * Trust in the distance sensors around the robot.
     */
    const float sensor_trust = 0.0;

    /**
     * Trust in the odometry system of the robot.
     */
    const unsigned char odometry_trust = 240;

    /**
     * Percentage at which the sensor values correct odometry as to smooth out correction
     */
    const float sensor_correction_gain = 0.25;
};

class localization_chassis;

/**
 * @brief Distance sensor wrapper class used for distance sensor resets and localization.
 */
class localization_sensor
{
    friend class localization_chassis;

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
     * Whether the display is active and being displayed.
     */
    bool b_display;

    /*
    * Active sensors being used by the robot.
    */
    int active_sensors;

    void set_active_sensors(int sensors);

public:

    /**
     * @brief Vectorizes acceleration from the inertial sensors, 3 axis accelerometer into a vector that only represents x and y acceleration
     * @param accel acceleration structure
     * @return acceleration vector
     */
    static vector2 vectorize(pros::imu_accel_s_t accel);

    /**
     * @brief Normalizes heading to the domain of 0-360.
     * @param heading heading to normalize.
     * @return Normalized heading
     */
    static float normalize_heading(float heading);

    /**
     * @breif Compares the location against locations the robot physically cannot exist at such as inside the match loader or out of bounds.
     *
     * @param pose current location vector
     * @returns whether the location can physically eist.
     */
    static bool can_position_exist(vector3 pose);

    static std::string get_quadrant_string(quadrant quad);

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
     * @breif Gets the robots quadrant based on its coordinates
     * @return The quadrant of the robot
     */
    quadrant get_quadrant();

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
    conf_pair<vector3> get_position_calculation(quadrant quad);

    conf_pair<vector3> get_position_calculation(quadrant quad0, quadrant quad1);

    /**
     * Initializes the debug screen.
     */
    static void init_display();

    /**
     * Renders the debug screen. Use in a loop.
     */
    static void update_display(localization_chassis* chassis);

    /**
     * Shutdown the debug screen
     */
    static void shutdown_display();

    /**
     * @breif Uses flags to return whether a sensor is being used.
     * @note Active sensors are set in the localization loop or by doing a distance sensor reset
     * @param r_sensor sensor flags
     * @return whether that sensor is being used.
     */
    bool is_sensor_used(int r_sensor)
    {
        return active_sensors & (r_sensor);
    }

private:

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
     * Provided options
     */
    localization_options options;

    /**
     * Last given localization data
     */
    localization_data data;

public:

    /**
     * @breif Initialize the localization chassis
     * @note ONLY INITIALIZE THIS WHEN YOUR ROBOT IS NOT MOVING!
     *
     * @param settings customizable trust and gain options for the localization algorithm
     * @param inertial pointer to the inertial sensor on the robot
     * @param base pointer to the lemlib chassis of the robot
     * @param sensors array of pointers to the localization sensors of the robot
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

    bool reset_location_normal(quadrant quad0, quadrant quad2);
};

#endif //PUSHBACK_LOCALIZATION_UTILS_HPP
