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

#include "../../include/pros/llemu.hpp"


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
 * Radian to Degree conversion factor
 */
static constexpr float rad_deg_conversion_factor = 57.2958;

/**
 * Distance to vex wall from origin in inches
 */
static constexpr float wall_coord = 70.208;

/**
 * Domain of the confidence readings from the V5 Distance Sensor
 */
static constexpr float confidence_domain = 63.0f;

static const rectangle north_goal_exclusion(-22, 44, 22, 49);
static const rectangle south_goal_exclusion(-22, -44, 22, -49);
static const rectangle mid_goal_exclusion(-7, -7, 7, -7);

static const circle pos_pos_ml_exclusion(67.5, 47, 5);
static const circle neg_pos_ml_exclusion(-67.5, 47, 5);
static const circle neg_neg_ml_exclusion(-67.5, -47, 5);
static const circle pos_neg_ml_exclusion(-67.5, 47, 5);

localization_sensor::localization_sensor(float off, int port) :
            offset(off),
            sensor(port)
{}

float errorize_heading_domain(float heading)
{
    if (static_cast<int>(heading) % 45 == 0) return 1;

    if (heading < -45.0f)
    {
        return errorize_heading_domain(heading + 90.0f);
    }

    if (heading > 45.0f)
    {
        return errorize_heading_domain(heading - 90.0f);
    }

    return heading;
}

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

    heading = errorize_heading_domain(heading);

    float heading_err_rad = heading * deg_rad_conversion_factor;

    float actual_reading = cos(heading_err_rad) * reading;
    float actual_offset = cos(heading_err_rad) * offset;

    if (heading == 270)
    {
        actual_offset = offset;
        actual_reading = reading;
    }

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

bool localization_chassis::can_position_exist(vector3 pose)
{
    vector2 loc = vector2(pose.x, pose.y);

    if (pose.x > wall_coord || pose.y > wall_coord || pose.x < -wall_coord || pose.y < -wall_coord) return false;

    if (pose.x > 0 && pose.y > 0)
    {
        return !(north_goal_exclusion.inside(loc) && mid_goal_exclusion.inside(loc) && pos_pos_ml_exclusion.inside(loc));
    }

    if (pose.x > 0 && pose.y < 0)
    {
        return !(north_goal_exclusion.inside(loc) && mid_goal_exclusion.inside(loc) && neg_pos_ml_exclusion.inside(loc));
    }

    if (pose.x < 0 && pose.y < 0)
    {
        return !(south_goal_exclusion.inside(loc) && mid_goal_exclusion.inside(loc) && neg_neg_ml_exclusion.inside(loc));
    }

    if (pose.x > 0 && pose.y < 0)
    {
        return !(south_goal_exclusion.inside(loc) && mid_goal_exclusion.inside(loc) && pos_neg_ml_exclusion.inside(loc));
    }

    return false;
}

std::string localization_chassis::get_quadrant_string(quadrant quad)
{
    switch (quad)
    {
        case POS_POS:
            return "POS_POS";
        case NEG_POS:
            return "NEG_POS";
        case NEG_NEG:
            return "NEG_NEG";
        case POS_NEG:
            return "POS_NEG";
    }
    return "";
}

void localization_chassis::set_active_sensors(int sensors)
{
    active_sensors = 0;
    active_sensors |= sensors;
}

vector2 localization_chassis::vectorize(pros::imu_accel_s_t accel)
{
    float magnitude = abs(sqrt(accel.x * accel.x + accel.y * accel.y));
    float angle = atan(abs(accel.x) / abs(accel.y)) * rad_deg_conversion_factor;

    if (accel.x > 0 && accel.y > 0)
    {
        angle = 1 * angle;
    } else if (accel.x < 0 && accel.y > 0)
    {
        angle = 180 - angle;
    } else if (accel.x < 0 && accel.y < 0)
    {
        angle = 180 + angle;
    } else if (accel.x > 0 && accel.y < 0)
    {
        angle = 360 - angle;
    }

    return vector2(magnitude,angle);
}

localization_chassis::localization_chassis(localization_options settings, pros::Imu *inertial, lemlib::Chassis* chas ,std::array<localization_sensor *,4> sensors) : options(
                                                                                                                                                                         settings), data(0, vector2(0,0), vector2(0,0), lemlib::Pose(0,0,0)), active_sensors(0), b_display(false)
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
    float heading = normalize_heading(chassis->getPose().theta);

    if ((heading >= 0 && heading <= 45) || (heading <= 360 && heading > 315))
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
//n_p, n_p
conf_pair<vector3> localization_chassis::get_position_calculation(quadrant quad)
{
    quadrant theta_quad = sensor_relevancy();

    return get_position_calculation(quad, theta_quad);
}

conf_pair<vector3> localization_chassis::get_position_calculation(quadrant quad, quadrant quad1)
{
    float normal_heading = normalize_heading(chassis->getPose().theta);
    quadrant theta_quad = quad1;

    conf_pair<float> n_dist = north->distance(normal_heading);
    conf_pair<float> e_dist = east->distance(normal_heading);
    conf_pair<float> s_dist = south->distance(normal_heading);
    conf_pair<float> w_dist = west->distance(normal_heading);

    conf_pair<vector3> ret = conf_pair<vector3>();

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
                break;
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(n_dist, w_dist));
                x = wall_coord - n_dist.get_value();
                y = wall_coord - w_dist.get_value();
                set_active_sensors(NORTH | WEST);
                break;
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(w_dist, s_dist));
                x = wall_coord - w_dist.get_value();
                y = wall_coord - s_dist.get_value();
                set_active_sensors(WEST | SOUTH);
                break;
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(s_dist, e_dist));
                x = wall_coord - s_dist.get_value();
                y = wall_coord - e_dist.get_value();
                set_active_sensors(SOUTH | EAST);
                break;
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
                break;
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + s_dist.get_value();
                y = wall_coord - w_dist.get_value();
                set_active_sensors(SOUTH | WEST);
                break;
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(e_dist, s_dist));
                x = -wall_coord + e_dist.get_value();
                y = wall_coord - s_dist.get_value();
                set_active_sensors(WEST | SOUTH);
                break;
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(e_dist, n_dist));
                x = -wall_coord + n_dist.get_value();
                y = wall_coord - e_dist.get_value();
                set_active_sensors(NORTH | EAST);
                break;
            }
        }
    } else if (quad == NEG_NEG)
    {
        switch (theta_quad)
        {
            case POS_POS:
            {
                ret.set_confidence(conf_avg(w_dist, s_dist));
                x = -wall_coord + w_dist.get_value();
                y = -wall_coord + s_dist.get_value();
                set_active_sensors(WEST | SOUTH);
                break;
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(s_dist, e_dist));
                x = -wall_coord + s_dist.get_value();
                y = -wall_coord + e_dist.get_value();
                set_active_sensors(SOUTH | EAST);
                break;
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(e_dist, n_dist));
                x = -wall_coord + e_dist.get_value();
                y = -wall_coord + n_dist.get_value();
                set_active_sensors(EAST | NORTH);
                break;
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(n_dist, w_dist));
                x = -wall_coord + n_dist.get_value();
                y = -wall_coord + w_dist.get_value();
                set_active_sensors(NORTH | WEST);
                break;
            }
        }
    } else if (quad == POS_NEG)
    {
        switch (theta_quad)
        {
            case POS_POS:
            {
                ret.set_confidence(conf_avg(e_dist, s_dist));
                x = -wall_coord + e_dist.get_value();
                y = -wall_coord + s_dist.get_value();
                set_active_sensors(EAST | SOUTH);
                break;
            }

            case NEG_POS:
            {
                ret.set_confidence(conf_avg(n_dist, e_dist));
                x = -wall_coord + n_dist.get_value();
                y = -wall_coord + e_dist.get_value();
                set_active_sensors(NORTH | EAST);
                break;
            }

            case NEG_NEG:
            {
                ret.set_confidence(conf_avg(w_dist, n_dist));
                x = -wall_coord + w_dist.get_value();
                y = -wall_coord + n_dist.get_value();
                set_active_sensors(WEST | NORTH);
                break;
            }

            case POS_NEG:
            {
                ret.set_confidence(conf_avg(s_dist, w_dist));
                x = -wall_coord + s_dist.get_value();
                y = -wall_coord + w_dist.get_value();
                set_active_sensors(SOUTH | WEST);
                break;
            }
        }
    } else
    {
        x = 0;
        y = 0;
        ret.set_confidence(0);
    }

    ret.set_value(vector3(x, y, normal_heading));

    if (!can_position_exist(vector3(x, y, normal_heading))) ret.set_confidence(0);

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
    conf_pair<vector3> coords = get_position_calculation(quad);

    if (coords.get_confidence() < options.sensor_trust) return false;

    pose.x = coords.get_value().x;
    pose.y = coords.get_value().y;
    chassis->setPose(pose);

    return true;
}

bool localization_chassis::reset_location_normal(quadrant quad0, quadrant quad2)
{
    lemlib::Pose pose = chassis->getPose();
    conf_pair<vector3> coords = get_position_calculation(quad0, quad2);

    if (coords.get_confidence() < options.sensor_trust) return false;

    pose.x = coords.get_value().x;
    pose.y = coords.get_value().y;
    chassis->setPose(pose);

    return true;
}

void localization_chassis::init_display()
{
    pros::lcd::initialize();
}

void localization_chassis::update_display(localization_chassis* chassis)
{
    bool north = chassis->is_sensor_used(NORTH);
    bool east = chassis->is_sensor_used(EAST);
    bool south = chassis->is_sensor_used(SOUTH);
    bool west = chassis->is_sensor_used(WEST);

    /*
    //float heading = normalize_heading(chassis->imu->get_heading());

    float confidence_n = chassis->north->distance(heading).get_confidence();
    float confidence_e = chassis->east->distance(heading).get_confidence();
    float confidence_s = chassis->south->distance(heading).get_confidence();
    float confidence_w = chassis->west->distance(heading).get_confidence();

    float dis_n = chassis->north->distance(heading).get_value();
    float dis_e = chassis->east->distance(heading).get_value();
    float dis_s = chassis->south->distance(heading).get_value();
    float dis_w = chassis->west->distance(heading).get_value();
    */

    //bool use_pose = chassis->reset_location_force(NEG_POS);
    //conf_pair<vector3> position = chassis->get_position_calculation(NEG_POS);
    //std::string quads = chassis->get_quadrant_string(chassis->get_quadrant());
    //std::string squad = chassis->get_quadrant_string(chassis->sensor_relevancy());

    lemlib::Pose pose_lem = chassis->chassis->getPose();

    //pros::lcd::print(0, "SQ: %s, %s", quads.c_str(), squad.c_str());
    pros::lcd::print(1, "SU: N %i, E %i, S %i, W %i", north, east, south, west);
    //pros::lcd::print(2, "SR: N %.2f, E %.2f, S %.2f, W %.2f", dis_n, dis_e, dis_s, dis_w);
    //pros::lcd::print(3, "SC: N %.2f, E %.2f, S %.2f, W %.2f", confidence_n, confidence_e, confidence_s, confidence_w);
    //pros::lcd::print(4, "PS: X: %.2f,Y: %.2f,H: %.2f,C: %.2f", position.get_value().x, position.get_value().y, heading, position.get_confidence());
    //pros::lcd::print(5, "LC: X: %.2f,Y: %.2f,H: %.2f,S: %i", pose_lem.x, pose_lem.y, pose_lem.theta, use_pose);
}

void localization_chassis::shutdown_display()
{
    pros::lcd::shutdown();
}





