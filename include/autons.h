//
// Created by aaron on 7/24/2025.
//

#ifndef AUTONS_H
#define AUTONS_H

#include "subsystems/conveyor.h"
#include "subsystems/drivetrain.h"

#include "titanselect/titanselect.hpp"

#define AUTON(name, routine) inline static ts::auton name = ts::auton(#name, []()routine)

AUTON(testing, {
    drivetrain* Drivetrain  = drivetrain::get();
    Drivetrain->lem_chassis.calibrate();
    Drivetrain->lem_chassis.moveToPoint(100, 100, 1000);
});

AUTON(BlueLeft, {
    drivetrain* Drivetrain  = drivetrain::get();
    Drivetrain->lem_chassis.calibrate();
});

AUTON(BlueRight, {
    drivetrain* Drivetrain  = drivetrain::get();
    Drivetrain->lem_chassis.calibrate();
});


#endif //AUTONS_H
