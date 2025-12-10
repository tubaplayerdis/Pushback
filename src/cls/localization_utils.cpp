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
#include "../../include/lemlib/pose.hpp"

static constexpr int err_reading_value = 9999;
static constexpr float mm_inch_conversion_factor = 0.0393701;
static constexpr float deg_rad_conversion_factor = 0.0174532;
static constexpr float wall_coord = 70.208;

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

conf_pair<float> localization_sensor::distance()
{
    int sensor_reading = sensor.get_distance();
    float sensor_confidence = (sensor.get_confidence() / 400.0f);
    if (sensor_reading == err_reading_value) return conf_pair<float>(err_reading_value, 0.0f);

    return conf_pair<float>(sensor_reading * mm_inch_conversion_factor, sensor_confidence);
}

conf_pair<float> localization_sensor::distance(float heading)
{
    int sensor_reading = sensor.get_distance();
    float sensor_confidence = (sensor.get_confidence() / 400.0f);
    if (sensor_reading == err_reading_value) return conf_pair<float>(err_reading_value, 0.0f);

    auto reading = sensor_reading * mm_inch_conversion_factor;

    float heading_err_rad = abs(heading - (90 * round(heading/90))) * deg_rad_conversion_factor;

    float actual_reading = cos(heading_err_rad) * reading;
    float actual_offset = cos(heading_err_rad) * offset;

    return conf_pair<float>(actual_reading + actual_offset, sensor_confidence);
}

localization_chassis::localization_chassis(localization_options settings, pros::Imu *inertial, lemlib::Chassis* chas ,std::array<localization_sensor *,4> sensors) : options(settings)
{
    north = sensors.at(0);
    east = sensors.at(1);
    south = sensors.at(2);
    west = sensors.at(3);
    imu = inertial;
    chassis = chas;

    last_call_time = pros::millis();
}

quadrant localization_chassis::sensor_relevancy()
{
    float heading = imu->get_heading();//TODO: Fix this to normalize the heading to the 0-360 domain

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

conf_pair<std::pair<float, float>> localization_chassis::get_position_calculation(quadrant quad)
{
    float normal_heading = imu->get_heading();//TODO: Fix this to normalize the heading to the 0-360 domain
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
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(n_dist, w_dist));
                x = wall_coord - n_dist.get_value();
                y = wall_coord - w_dist.get_value();
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(w_dist, s_dist));
                x = wall_coord - w_dist.get_value();
                y = wall_coord - s_dist.get_value();
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(s_dist, e_dist));
                x = wall_coord - s_dist.get_value();
                y = wall_coord - e_dist.get_value();
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
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + s_dist.get_value();
                y = wall_coord - w_dist.get_value();
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + w_dist.get_value();
                y = wall_coord - s_dist.get_value();
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(e_dist, n_dist));
                x = -wall_coord + n_dist.get_value();
                y = wall_coord - e_dist.get_value();
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
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + s_dist.get_value();
                y = -wall_coord + e_dist.get_value();
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + e_dist.get_value();
                y = -wall_coord + n_dist.get_value();
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(e_dist, n_dist));
                x = -wall_coord + n_dist.get_value();
                y = -wall_coord + w_dist.get_value();
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
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + n_dist.get_value();
                y = -wall_coord + e_dist.get_value();
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + w_dist.get_value();
                y = -wall_coord + n_dist.get_value();
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(e_dist, n_dist));
                x = -wall_coord + s_dist.get_value();
                y = -wall_coord + w_dist.get_value();
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

unsigned char localization_chassis::conf_avg(conf_pair<float> one, conf_pair<float> two)
{
    return static_cast<unsigned char>((static_cast<int>(one.get_confidence()) + static_cast<int>(two.get_confidence())) / 2);
}

bool localization_chassis::reset_location()
{
    return false;
}

bool localization_chassis::reset_location_force(quadrant quad)
{

    lemlib::Pose pose = chassis->getPose();

    float normal_heading = pose.theta;


    quadrant theta_quad = sensor_relevancy(normal_heading);

    bool value_pass = false;

    conf_pair<float> n_dist = north->distance(normal_heading);
    conf_pair<float> e_dist = east->distance(normal_heading);
    conf_pair<float> s_dist = south->distance(normal_heading);
    conf_pair<float> w_dist = west->distance(normal_heading);

    float x = 0;
    float y = 0;

    if (quad == POS_POS)
    {
        switch (theta_quad)
        {
            case POS_POS:
            {
                value_pass = value_check(options, e_dist, n_dist);
                x = wall_coord - e_dist.get_value();
                y = wall_coord - n_dist.get_value();
            }

            case NEG_POS:
            {
                value_pass = value_check(options, n_dist, w_dist);
                x = wall_coord - n_dist.get_value();
                y = wall_coord - w_dist.get_value();
            }

            case NEG_NEG:
            {
                value_pass = value_check(options, w_dist, s_dist);
                x = wall_coord - w_dist.get_value();
                y = wall_coord - s_dist.get_value();
            }

            case POS_NEG:
            {
                value_pass = value_check(options, s_dist, e_dist);
                x = wall_coord - s_dist.get_value();
                y = wall_coord - e_dist.get_value();
            }
        }

        if (!value_pass) return false;

    }

    pose.x = x;
    pose.y = y;

    chassis->setPose(pose);

    return true;
}


