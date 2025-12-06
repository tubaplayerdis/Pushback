//
// Created by aaron on 11/22/2025.
//

#include "../../include/cls/localization_utils.hpp"
#include "../../include/pros/rtos.hpp"
#include <optional>

#include "../../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/optional"
#include "../../include/subsystems/drivetrain.hpp"

static constexpr int err_reading_value = 9999;
static constexpr float mm_inch_conversion_factor = 0.0393701;
static constexpr float deg_rad_conversion_factor = 0.0174532;

vector::vector()
{
    x = 0;
    y = 0;
    z = 0;
}

vector::vector(float X, float Y, float Z)
{
    x = X;
    y = Y;
    z = Z;
}

localization_sensor::localization_sensor(float off, int port) :
            offset(off),
            sensor(port)
{}

std::optional<float> localization_sensor::distance()
{
    int sensor_reading = sensor.get_distance();
    if (sensor_reading == err_reading_value) return std::nullopt;

    return (float)sensor_reading * mm_inch_conversion_factor;
}

std::optional<float> localization_sensor::distance(float heading)
{
    int sensor_reading = sensor.get_distance();
    if (sensor_reading == err_reading_value) return std::nullopt;

    auto reading = sensor_reading * mm_inch_conversion_factor;

    float heading_err_rad = abs(heading - (90 * round(heading/90))) * deg_rad_conversion_factor;

    float actual_reading = cos(heading_err_rad) * reading;
    float actual_offset = cos(heading_err_rad) * offset;

    return actual_reading + actual_offset;
}

float localization_sensor::distance_raw()
{
    std::optional<float> dis = distance();
    if (dis) return dis.value();
    return 9999;
}

localization_chassis::localization_chassis(pros::Imu *inertial, std::array<localization_sensor *, 4> sensors)
{
    north = sensors.at(0);
    east = sensors.at(1);
    south = sensors.at(2);
    west = sensors.at(3);
    imu = inertial;
}

