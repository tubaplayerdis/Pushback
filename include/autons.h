//
// Created by aaron on 7/24/2025.
//

#ifndef AUTONS_H
#define AUTONS_H

#include "conveyor.h"
#include "drivetrain.h"
#include "cls/auton.h"

AUTON(testing, {
    Drivetrain->Chassis.calibrate();
    Drivetrain->Chassis.moveToPoint(100, 100, 1000);
});

AUTON(BlueLeft, {
    Drivetrain->Chassis.calibrate();
});

#endif //AUTONS_H
