//
// Created by aaron on 7/23/2025.
//

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "cls/subsystem.h"
#include "pros/motor_group.hpp"
#include "controller.h"
#include "odometry.h"


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

#define DRIVETRAIN_TRACK_WIDTH 10
#define DRIVETRAIN_WHEEL_DIAMETER lemlib::Omniwheel::NEW_325
#define DRIVETRAIN_RPM 450
#define DRIVETRAIN_HORIZONTAL_DRIFT 2
#define DRIVETRAIN_MOTOR_CARTRIDGE pros::v5::MotorGears::blue

#define DRIVETRAIN drivetrain::Get()

class drivetrain final : public subsystem
{
public:
    pros::MotorGroup MotorsLeft;
    pros::MotorGroup MotorsRight;
    lemlib::Drivetrain LemDrivetrain;
    lemlib::Chassis Chassis;

private:
    drivetrain() :
    MotorsLeft({PORT_LEFT_A, PORT_LEFT_B, PORT_LEFT_C}, DRIVETRAIN_MOTOR_CARTRIDGE),
    MotorsRight({PORT_RIGHT_A, PORT_RIGHT_B, PORT_RIGHT_C}, DRIVETRAIN_MOTOR_CARTRIDGE) ,
    LemDrivetrain(&MotorsLeft, &MotorsRight, DRIVETRAIN_TRACK_WIDTH, DRIVETRAIN_WHEEL_DIAMETER, DRIVETRAIN_RPM, DRIVETRAIN_HORIZONTAL_DRIFT),
    Chassis(LemDrivetrain, controller::ControllerSettingsLateral, controller::ControllerSettingsAngular, ODOMETRY->OdometrySensors, &controller::ExpoCurveThrottle, &controller::ExpoCurveSteer)
    {
        Chassis.calibrate();
        Chassis.setPose(0,0,0);//Set the local location controller to zero
    }

protected:
    void Tick_Implementation() override;

public:
    static drivetrain* Get();
};

#endif //DRIVETRAIN_H
