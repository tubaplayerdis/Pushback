//
// Created by aaron on 7/24/2025.
//

#ifndef AUTONS_H
#define AUTONS_H

#include "conveyor.h"
#include "drivetrain.h"
#include "titanselect/titanselect.hpp"

AUTON(testing, {
    DRIVETRAIN->Chassis.calibrate();
    DRIVETRAIN->Chassis.moveToPoint(100, 100, 1000);
});

AUTON(BlueLeft, {
    DRIVETRAIN->Chassis.calibrate();
});

#endif //AUTONS_H
