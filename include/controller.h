//
// Created by aaron on 7/20/2025.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "pros/misc.hpp"
#include "lemlib/chassis/chassis.hpp"

#define CONTROLLER_VERTICAL_AXIS pros::E_CONTROLLER_ANALOG_LEFT_Y
#define CONTROLLER_HORIZONTAL_AXIS pros::E_CONTROLLER_ANALOG_RIGHT_X

namespace controller
{
    inline lemlib::ControllerSettings
    ControllerSettingsLateral(8,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       4,   // derivative gain (kD)
                       3,   // anti-windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

    // angular PID controller
    inline lemlib::ControllerSettings
    ControllerSettingsAngular(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,  // derivative gain (kD)
                       3,   // anti-windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

    // input curve for throttle input during driver control
    inline lemlib::ExpoDriveCurve
    ExpoCurveThrottle(3,    // joystick deadband out of 127
                   10,   // minimum output where drivetrain will move out of 127
                   1.019 // expo curve gain
    );

    // input curve for steer input during driver control
    inline lemlib::ExpoDriveCurve
    ExpoCurveSteer(3,    // joystick deadband out of 127
                10,   // minimum output where drivetrain will move out of 127
                1.035 // expo curve gain
    );
}

inline pros::Controller Controller(pros::E_CONTROLLER_MASTER);

#endif //CONTROLLER_H
