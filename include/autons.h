//
// Created by aaron on 7/24/2025.
//

#ifndef AUTONS_H
#define AUTONS_H

#include "subsystems/conveyor.h"
#include "subsystems/drivetrain.h"

#include "titanselect/titanselect.hpp"

AUTON(testing, {
    drivetrain* Drivetrain  = drivetrain::Get();
    Drivetrain->Chassis.calibrate();
    Drivetrain->Chassis.moveToPoint(100, 100, 1000);
});

AUTON(BlueLeft, {
    drivetrain* Drivetrain  = drivetrain::Get();
    Drivetrain->Chassis.calibrate();
});

AUTON(BlueRight, {
    drivetrain* Drivetrain  = drivetrain::Get();
    Drivetrain->Chassis.calibrate();
});


#endif //AUTONS_H
