//
// Created by aaron on 7/20/2025.
//

#ifndef CONVEYOR_H
#define CONVEYOR_H

#include "../cls/subsystem.h"
#include "../pros/adi.hpp"
#include "../pros/motor_group.hpp"
#include "../pros/misc.h"
#include "../controller.h"
#include "../pros/optical.hpp"

enum ColorType
{
    NEUTRAL = 0,
    BLUE = 1,
    RED = 2
};

class conveyor final : public subsystem
{
    friend class std::unique_ptr<conveyor>;

public:
    pros::Motor Intake;
    pros::Motor Exhaust;
    pros::MotorGroup ConveyorGroup; //Runs the intake and other system requiring the path of movement.
    pros::Optical SplitterOptical;
    pros::adi::Pneumatics Splitter; //Dictates whether balls are scored/thrown.
    std::unique_ptr<pros::Task> ColorSortTask;
    ColorType ColorSortStatus;

private:
    conveyor();

public:
    void ActiveColorSort();
    void DeactivateColorSort();

protected:
    void tick_implementation() override;

public:
    static conveyor* get();
};

#endif //CONVEYOR_H
