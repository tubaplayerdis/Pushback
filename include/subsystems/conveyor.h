//
// Created by aaron on 7/20/2025.
//

#ifndef CONVEYOR_H
#define CONVEYOR_H

#include "../cls/subsystem.h"
#include "../pros/adi.hpp"
#include "../pros/motor_group.hpp"
#include "../pros/misc.h"
#include "../monitor.h"
#include "../controller.h"
#include "../cpid.h"
#include "../pros/optical.hpp"

//Port macros
#define PORT_INTAKE 0
#define PORT_EXHAUST 0
#define PORT_NORMAL_A 1
#define PORT_NORMAL_B -10
#define PORT_SPLITTER 'A'
#define PORT_SPLITTER_OPTICAL 12

//Control macros
#define CONVEYOR_IN pros::E_CONTROLLER_DIGITAL_L1
#define CONVEYOR_OUT pros::E_CONTROLLER_DIGITAL_R1

constexpr auto EXHAUST_OUT = pros::E_CONTROLLER_DIGITAL_R2;

class conveyor final : public subsystem
{
public:
    pros::Motor Intake;
    pros::Motor Exhaust;
    pros::MotorGroup ConveyorGroup; //Runs the intake and other system requiring the path of movement.
    pros::Optical SplitterOptical;
    pros::adi::Pneumatics Splitter; //Dictates whether balls are scored/thrown.
    bool Inverted;

public:
    conveyor() :
    Intake(PORT_INTAKE),
    Exhaust(PORT_EXHAUST),
    ConveyorGroup({PORT_NORMAL_A, PORT_NORMAL_B}),
    SplitterOptical(PORT_SPLITTER_OPTICAL),
    Splitter(PORT_SPLITTER, false), Inverted(false) {}

protected:
    void Tick_Implementation() override;

public:
    static conveyor* Get();
};

#endif //CONVEYOR_H
