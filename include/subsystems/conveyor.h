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
constexpr auto PORT_INTAKE = 0;
constexpr auto PORT_EXHAUST = 0;
constexpr auto PORT_NORMAL_A = 1;
constexpr auto PORT_NORMAL_B = -10;
constexpr auto PORT_SPLITTER = 'A';
constexpr auto PORT_SPLITTER_OPTICAL = 12;

//Control macros
constexpr auto CONVEYOR_IN = pros::E_CONTROLLER_DIGITAL_L1;
constexpr auto CONVEYOR_OUT = pros::E_CONTROLLER_DIGITAL_R1;
constexpr auto EXHAUST_OUT = pros::E_CONTROLLER_DIGITAL_R2;

enum ColorType
{
    NEUTRAL = 0,
    BLUE = 1,
    RED = 2
};

class conveyor final : public subsystem
{
public:
    pros::Motor Intake;
    pros::Motor Exhaust;
    pros::MotorGroup ConveyorGroup; //Runs the intake and other system requiring the path of movement.
    pros::Optical SplitterOptical;
    pros::adi::Pneumatics Splitter; //Dictates whether balls are scored/thrown.
    std::unique_ptr<pros::Task> ColorSortTask;
    ColorType ColorSortStatus;

public:
    conveyor() :
    Intake(PORT_INTAKE),
    Exhaust(PORT_EXHAUST),
    ConveyorGroup({PORT_NORMAL_A, PORT_NORMAL_B}),
    SplitterOptical(PORT_SPLITTER_OPTICAL),
    Splitter(PORT_SPLITTER, false),
    ColorSortTask(nullptr),
    ColorSortStatus(ColorType::NEUTRAL)
    {
        SplitterOptical.set_led_pwm(50); //50% brightness
    };

public:
    void ActiveColorSort();
    void DeactivateColorSort();

protected:
    void Tick_Implementation() override;

public:
    static conveyor* Get();
};

#endif //CONVEYOR_H
