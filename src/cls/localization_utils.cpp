//
// Created by aaron on 11/22/2025.
//

#include "../../include/cls/localization_utils.hpp"

#include <array>

#include "../../include/pros/rtos.hpp"
#include <optional>
#include <vector>

#include "../../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/optional"
#include "../../include/subsystems/drivetrain.hpp"
#include "../../include/liblvgl/lvgl.h"
#include "../../include/lemlib/pose.hpp"
#include "../../include/pros/imu.h"
#include "../../include/pros/imu.hpp"
#include <cmath>

#include "../../include/images/images.hpp"

/**
 * Erroneous reading value when the V5 Distance Sensor cannot read a distance
 */
static constexpr int err_reading_value = 9999;

/**
 * MM to IN conversion factor.
 */
static constexpr float mm_inch_conversion_factor = 0.0393701;

/**
 * Degree to Radian conversion factor
 */
static constexpr float deg_rad_conversion_factor = 0.0174532;

/**
 * Distance to vex wall from origin in inches
 */
static constexpr float wall_coord = 70.208;

/**
 * Domain of the confidence readings from the V5 Distance Sensor
 */
static constexpr float confidence_domain = 63.0f;

localization_sensor::localization_sensor(float off, int port) :
            offset(off),
            sensor(port)
{}

conf_pair<float> localization_sensor::distance()
{
    int sensor_reading = sensor.get_distance();
    float sensor_confidence = (sensor.get_confidence() / confidence_domain);
    if (sensor_reading == err_reading_value) return conf_pair<float>(err_reading_value, 0.0f);

    return conf_pair<float>(sensor_reading * mm_inch_conversion_factor, sensor_confidence);
}

conf_pair<float> localization_sensor::distance(float heading)
{
    int sensor_reading = sensor.get_distance();
    float sensor_confidence = (sensor.get_confidence() / confidence_domain);
    if (sensor_reading == err_reading_value) return conf_pair<float>(err_reading_value, 0.0f);

    auto reading = sensor_reading * mm_inch_conversion_factor;

    float heading_err_rad = std::abs(heading - (90 * round(heading/90))) * deg_rad_conversion_factor;

    float actual_reading = cos(heading_err_rad) * reading;
    float actual_offset = cos(heading_err_rad) * offset;

    return conf_pair<float>(actual_reading + actual_offset, sensor_confidence);
}

float localization_chassis::normalize_heading(float heading)
{
    if (heading < 0.0f)
    {
        return normalize_heading(heading + 360.0f);
    }

    if (heading > 360.0f)
    {
        return normalize_heading(heading - 360.0f);
    }

    return heading;
}

void localization_chassis::set_active_sensors(int sensors)
{
    active_sensors = 0;
    active_sensors = sensors;
}

localization_chassis::localization_chassis(localization_options settings, pros::Imu *inertial, lemlib::Chassis* chas ,std::array<localization_sensor *,4> sensors) : options(
    settings), data(0, vector2(0,0), vector2(0,0), lemlib::Pose(0,0,0)), active_sensors(0)
{
    north = sensors.at(0);
    east = sensors.at(1);
    south = sensors.at(2);
    west = sensors.at(3);
    imu = inertial;
    chassis = chas;

    auto pose = chassis->getPose();

    pros::imu_accel_s_t accel = imu->get_accel();
    float heading = imu->get_heading();

    localization_data current = {
        pros::millis(), vector2(accel.x, accel.y), vector2(0, heading), pose
    };
    data = current;
}

quadrant localization_chassis::sensor_relevancy()
{
    float heading = normalize_heading(imu->get_heading());

    if ((heading > 0 && heading <= 45) || (heading <= 360 && heading > 315))
    {
        return POS_POS;
    }

    if (heading > 45 && heading <= 135)
    {
        return NEG_POS;
    }

    if (heading > 135 && heading <= 225)
    {
        return NEG_NEG;
    }

    if (heading > 225 && heading <= 315)
    {
        return POS_NEG;
    }

    return NEG_NEG;
}

quadrant localization_chassis::get_quadrant()
{
    lemlib::Pose cur_pose = chassis->getPose();

    if (cur_pose.x > 0 && cur_pose.y > 0)
    {
        return POS_POS;
    }

    if (cur_pose.x > 0 && cur_pose.y < 0)
    {
        return NEG_POS;
    }

    if (cur_pose.x < 0 && cur_pose.y < 0)
    {
        return NEG_NEG;
    }

    if (cur_pose.x > 0 && cur_pose.y < 0)
    {
        return POS_NEG;
    }

    return POS_POS;
}

conf_pair<std::pair<float, float>> localization_chassis::get_position_calculation(quadrant quad)
{
    float normal_heading = normalize_heading(imu->get_heading());
    quadrant theta_quad = sensor_relevancy();

    conf_pair<float> n_dist = north->distance(normal_heading);
    conf_pair<float> e_dist = east->distance(normal_heading);
    conf_pair<float> s_dist = south->distance(normal_heading);
    conf_pair<float> w_dist = west->distance(normal_heading);

    conf_pair<std::pair<float, float>> ret = conf_pair<std::pair<float, float>>();

    float x = 0;
    float y = 0;

    if (quad == POS_POS)
    {
        switch (theta_quad)
        {
            case POS_POS:
            {
                ret.set_confidence(conf_avg(e_dist, n_dist));
                x = wall_coord - e_dist.get_value();
                y = wall_coord - n_dist.get_value();
                set_active_sensors(NORTH | EAST);
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(n_dist, w_dist));
                x = wall_coord - n_dist.get_value();
                y = wall_coord - w_dist.get_value();
                set_active_sensors(NORTH | WEST);
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(w_dist, s_dist));
                x = wall_coord - w_dist.get_value();
                y = wall_coord - s_dist.get_value();
                set_active_sensors(WEST | SOUTH);
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(s_dist, e_dist));
                x = wall_coord - s_dist.get_value();
                y = wall_coord - e_dist.get_value();
                set_active_sensors(SOUTH | EAST);
            }
        }
    } else if (quad == NEG_POS)
    {
        switch (theta_quad)
        {
            case POS_POS:
            {
                ret.set_confidence(conf_avg(n_dist, w_dist));
                x = -wall_coord + w_dist.get_value();
                y = wall_coord - n_dist.get_value();
                set_active_sensors(WEST | NORTH);
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + s_dist.get_value();
                y = wall_coord - w_dist.get_value();
                set_active_sensors(SOUTH | WEST);
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + w_dist.get_value();
                y = wall_coord - s_dist.get_value();
                set_active_sensors(WEST | SOUTH);
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(e_dist, n_dist));
                x = -wall_coord + n_dist.get_value();
                y = wall_coord - e_dist.get_value();
                set_active_sensors(NORTH | EAST);
            }
        }
    } else if (quad == NEG_NEG)
    {
        switch (theta_quad)
        {
            case POS_POS:
            {
                ret.set_confidence(conf_avg(n_dist, w_dist));
                x = -wall_coord + w_dist.get_value();
                y = -wall_coord + s_dist.get_value();
                set_active_sensors(WEST | SOUTH);
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + s_dist.get_value();
                y = -wall_coord + e_dist.get_value();
                set_active_sensors(SOUTH | EAST);
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + e_dist.get_value();
                y = -wall_coord + n_dist.get_value();
                set_active_sensors(EAST | NORTH);
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(e_dist, n_dist));
                x = -wall_coord + n_dist.get_value();
                y = -wall_coord + w_dist.get_value();
                set_active_sensors(NORTH | WEST);
            }
        }
    } else if (quad == POS_NEG)
    {
        switch (theta_quad)
        {
            case POS_POS:
            {
                ret.set_confidence(conf_avg(n_dist, w_dist));
                x = -wall_coord + e_dist.get_value();
                y = -wall_coord + s_dist.get_value();
                set_active_sensors(EAST | SOUTH);
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + n_dist.get_value();
                y = -wall_coord + e_dist.get_value();
                set_active_sensors(NORTH | EAST);
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + w_dist.get_value();
                y = -wall_coord + n_dist.get_value();
                set_active_sensors(WEST | NORTH);
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(e_dist, n_dist));
                x = -wall_coord + s_dist.get_value();
                y = -wall_coord + w_dist.get_value();
                set_active_sensors(SOUTH | WEST);
            }
        }
    } else
    {
        x = 0;
        y = 0;
        ret.set_confidence(0);
    }

    ret.set_value(std::pair<float, float>(x, y));

    return ret;
}

float localization_chassis::conf_avg(conf_pair<float> one, conf_pair<float> two)
{
    return (one.get_confidence() + two.get_confidence()) / 2.0f;
}

bool localization_chassis::reset_location()
{
    return reset_location_force(get_quadrant());
}

bool localization_chassis::reset_location_force(quadrant quad)
{

    lemlib::Pose pose = chassis->getPose();
    conf_pair<std::pair<float, float>> coords = get_position_calculation(quad);

    if (coords.get_confidence() < options.sensor_trust) return false;

    pose.x = coords.get_value().first;
    pose.y = coords.get_value().second;
    chassis->setPose(pose);

    return true;
}


static lv_obj_t* field_image = nullptr;

void localization_chassis::init_display()
{
    field_image = lv_image_create(lv_screen_active());
    lv_image_set_src(field_image, VexField240x240_map);
}

void localization_chassis::display_debug()
{
}



