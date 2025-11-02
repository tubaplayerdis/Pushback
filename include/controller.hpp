//
// Created by aaron on 7/20/2025.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "pros/misc.hpp"
#include "lemlib/chassis/chassis.hpp"

namespace controller
{
    // input curve for throttle input during driver control
    inline lemlib::ExpoDriveCurve
    expo_curve_throttle(3,    // joystick deadband out of 127
                   10,   // minimum output where drivetrain will move out of 127
                   1.019 // expo curve gain
    );

    // input curve for steer input during driver control
    inline lemlib::ExpoDriveCurve
    expo_curve_steer(3,    // joystick deadband out of 127
                10,   // minimum output where drivetrain will move out of 127
                1.035 // expo curve gain
    );
}

inline pros::Controller controller_master(pros::E_CONTROLLER_MASTER);

#endif //CONTROLLER_H
