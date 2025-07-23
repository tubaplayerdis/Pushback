//
// Created by aaron on 7/23/2025.
//

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "cls/subsystem.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/misc.h"
#include "monitor.h"

//Port macros
#define PORT_LEFT_A -19
#define PORT_LEFT_B 15
#define PORT_LEFT_C 16
#define PORT_RIGHT_A 17
#define PORT_RIGHT_B -18
#define PORT_RIGHT_C -20

//Control macros
#define VERTICAL_AXIS pros::E_CONTROLLER_ANALOG_LEFT_Y
#define HORIZONTAL_AXIS pros::E_CONTROLLER_ANALOG_RIGHT_X

class drivetrain : public subsystem
{
    public:
    pros::MotorGroup MotorsLeft;
    pros::MotorGroup MotorsRight;

    drivetrain() : subsystem(), MotorsLeft({PORT_LEFT_A, PORT_LEFT_B, PORT_LEFT_C}), MotorsRight({PORT_RIGHT_A, PORT_RIGHT_B, PORT_RIGHT_C}) {}

    void Tick() override;
};

inline void drivetrain::Tick()
{
    //Handle moving the drivetrain.
}

#endif //DRIVETRAIN_H
