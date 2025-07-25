//
// Created by aaron on 7/24/2025.
//

#ifndef TESTING_H
#define TESTING_H
#include "../drivetrain.h"
#include "../cls/auton.h"

class testing : public auton
{
public:
    testing() : auton([]()
    {
        //Write the auton here
        Drivetrain->Chassis.moveToPoint(100, 100, 0);
    }) {}
};

#endif //TESTING_H
