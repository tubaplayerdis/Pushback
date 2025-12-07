//
// Created by aaron on 11/22/2025.
//

#include "../../include/cls/localization_utils.hpp"

#include <array>

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

probability localization_sensor::distance()
{
    int sensor_reading = sensor.get_distance();
    unsigned char sensor_confidence = static_cast<unsigned char>(static_cast<float>(sensor.get_confidence()) / 400.0f * 255.0f);
    if (sensor_reading == err_reading_value) return probability(err_reading_value, 0);

    return probability(sensor_reading * mm_inch_conversion_factor, sensor_confidence);
}

probability localization_sensor::distance(float heading)
{
    int sensor_reading = sensor.get_distance();
    unsigned char sensor_confidence = static_cast<unsigned char>(static_cast<float>(sensor.get_confidence()) / 400.0f * 255.0f);
    if (sensor_reading == err_reading_value) return probability(err_reading_value, 0);

    auto reading = sensor_reading * mm_inch_conversion_factor;

    float heading_err_rad = abs(heading - (90 * round(heading/90))) * deg_rad_conversion_factor;

    float actual_reading = cos(heading_err_rad) * reading;
    float actual_offset = cos(heading_err_rad) * offset;

    return probability(actual_reading + actual_offset, sensor_confidence);
}

localization_chassis::localization_chassis(pros::Imu *inertial, lemlib::Chassis* chas ,std::array<localization_sensor *,4> sensors)
{
    north = sensors.at(0);
    east = sensors.at(1);
    south = sensors.at(2);
    west = sensors.at(3);
    imu = inertial;
    chassis = chas;

    last_call_time = pros::millis();
}

bool localization_chassis::reset_location(bool ignore_probability, int sensors)
{
    return false;
}


