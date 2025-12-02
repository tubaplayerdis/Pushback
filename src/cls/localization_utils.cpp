//
// Created by aaron on 11/22/2025.
//

#include "../../include/cls/localization_utils.hpp"
#include "../../include/pros/rtos.hpp"
#include <optional>

#include "../../include/subsystems/drivetrain.hpp"

static constexpr int err_reading_value = 9999;
static constexpr float mm_inch_conversion_factor = 0.0393701;

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

localization_sensor::localization_sensor(vector off, vector::axis axis, int port) :
            offset(off),
            localization_axis(axis),
            sensor(port),
            sensor_model({off.x, off.y, off.z}, sensor) {}

std::optional<float> localization_sensor::distance()
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

float localization_sensor::distance_raw()
{
    std::optional<float> dis = distance();
    if (dis) return dis.value();
    return 9999;
}


loco::DistanceSensorModel* localization_sensor::get_sensor_model()
{
    return &sensor_model;
}

localization_data::localization_data() :
    random_gen(pros::millis()),
    odom_change(0.0),
    last_odom(0.0),
    last_theta(0.0),
    exponential_pose(Eigen::Vector3f::Zero())
{}
