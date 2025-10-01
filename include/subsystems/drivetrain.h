//
// Created by aaron on 7/23/2025.
//

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <memory>

#include "../cls/subsystem.h"
#include "../pros/motor_group.hpp"
#include "../pros/distance.hpp"
#include "../controller.h"
#include "ports.h"
#include "odometry.h"

struct localization_sensor
{
    const double x_offset; //Inches. measured from back of robot where the distance is the absolute value from the center x of the robot. this should be positive
    const double y_offset; //Inches. measured from back of robot where the distance is a positive amount from the center.
    pros::Distance distance_sensor;

    localization_sensor(const double x_offset, const double y_offset, char port) : x_offset{x_offset}, y_offset{y_offset}, distance_sensor(port)
    {}
};

/// drivetrain class represents the physical drivetrain of the robot. inherits subsystem
class drivetrain final : public subsystem
{
    /// Friend class to allow unique_ptr to access deconstructor
    friend class std::unique_ptr<drivetrain>;

public:

    /// Left motor group. Includes 3 motors, A, B, C
    pros::MotorGroup motors_left;

    /// Right motor group. Includes 3 motors, A, B, C
    pros::MotorGroup motors_right;

    /// LemLib drivetrain object. Used in autons and the chassis object
    lemlib::Drivetrain lem_drivetrain;

    /// LemLib chassis object. Used in autons and for controller inputs
    lemlib::Chassis lem_chassis;

    /// Localization (Distance) sensor on the back of the robot.
    localization_sensor localization_rear;

    /// Localization (Distance) sensor on the left of the robot.
    localization_sensor localization_left;

    /// Localization (Distance) sensor on the right of the robot.
    localization_sensor localization_right;

private:

    /// private constructor to enforce the usage of the get() function.
    /// creates the drivetrain object and initializes all the class variables.
    drivetrain();

    /// Gets the x position of the robot in inches.
    /// @return distance from the wall of the robot accounting for the offset of the distance sensor itself from the center of the robot.
    double get_x_position_from_sensors();

    /// Gets the y position of the robot in inches.
    /// @return distance from the wall of the robot accounting for the offset of the distance sensor itself from the center of the robot.
    double get_y_position_from_sensor();

protected:

    /// gathers controller info and relays that to the drivetrain.
    void tick_implementation() override;

public:

    /// public static accessor function to access the drivetrain singleton. creates the drivetrain object if the singleton is null.
    /// \return raw pointer to the drivetrain object.
    static drivetrain* get();
};

#endif //DRIVETRAIN_H
