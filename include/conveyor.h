//
// Created by aaron on 7/20/2025.
//

#ifndef CONVEYOR_H
#define CONVEYOR_H

#include "cls/subsystem.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/misc.h"
#include "monitor.h"
#include "controller.h"

//Port macros
#define PORT_NORMAL_A -1
#define PORT_NORMAL_B -1
#define PORT_INVERTED_A -1
#define PORT_INVERTED_B -1
#define PORT_SPLITTER 'A'

//Control macros
#define CONVEYOR_IN pros::E_CONTROLLER_DIGITAL_L1
#define CONVEYOR_OUT pros::E_CONTROLLER_DIGITAL_R1

class conveyor final : public subsystem
{
public:
    pros::MotorGroup NormalGroup; //Runs the intake and other system requiring the path of movement.
    pros::MotorGroup InvertedGroup; //Mostly responsible for getting the blocks into storage
    pros::adi::Pneumatics Splitter; //Dictates whether balls are stowed/scored.

    conveyor() : subsystem(false, false), NormalGroup({PORT_NORMAL_A, PORT_NORMAL_B}), InvertedGroup({PORT_INVERTED_A, PORT_INVERTED_B}), Splitter(PORT_SPLITTER, false) {}

private:
    bool Activate_Implementation() override;
    bool Deactivate_Implementation() override;

public:
    void Tick() override;
};

inline bool conveyor::Activate_Implementation()
{
    return true;
}

inline bool conveyor::Deactivate_Implementation()
{
    return true;
}

inline void conveyor::Tick()
{
    if (Controller.get_digital(CONVEYOR_IN))
    {
        Handle(NormalGroup.move(FULL_POWER));
        Handle(InvertedGroup.move(-FULL_POWER));
    } else if (Controller.get_digital(CONVEYOR_OUT))
    {
        Handle(NormalGroup.move(-FULL_POWER));
        Handle(InvertedGroup.move(FULL_POWER));
    } else
    {
        Handle(NormalGroup.brake());
        Handle(InvertedGroup.brake());
    }
}

#endif //CONVEYOR_H
