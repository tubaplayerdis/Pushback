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
#include "cpid.h"

//Port macros
#define PORT_NORMAL_A 1
#define PORT_NORMAL_B -10
#define PORT_NORMAL_C (-9)
#define PORT_NORMAL_D 2
#define PORT_SPLITTER 'A'

//Control macros
#define CONVEYOR_IN pros::E_CONTROLLER_DIGITAL_L1
#define CONVEYOR_OUT pros::E_CONTROLLER_DIGITAL_R1

#define SCORE_TOP pros::E_CONTROLLER_DIGITAL_L2
#define SCORE_BOTTOM pros::E_CONTROLLER_DIGITAL_R2

#define CONVEYOR conveyor::Get()

class conveyor final : public subsystem
{
    inline static conveyor* instance;
public:
    cpid PID;
    pros::MotorGroup NormalGroup; //Runs the intake and other system requiring the path of movement.
    pros::MotorGroup InvertedGroup; //Scoring Flexwheels
    pros::adi::Pneumatics Splitter; //Dictates whether balls are stowed/scored.
    bool Inverted;

private:
    //The constructor is private so the only way to access the conveyor is via the Conveyor macro
    conveyor() :
    NormalGroup({PORT_NORMAL_A, PORT_NORMAL_B, PORT_NORMAL_D}),
    InvertedGroup({PORT_NORMAL_C}),
    Splitter(PORT_SPLITTER, false), Inverted(false) {}

protected:
    void Tick_Implementation() override;

public:
    static conveyor* Get();
};

inline conveyor* conveyor::Get()
{
    if (!instance) instance = new conveyor();
    return instance;
}

//The idea here is that we are trying to reach 500 rpm when the system is activated and it will auto power up to that speed. This is done to prevent overheating.
inline void conveyor::Tick_Implementation()
{
    double normSpeed = NormalGroup.get_actual_velocity();
    double invSpeed = InvertedGroup.get_actual_velocity();
    float avgSpeed = static_cast<float>((normSpeed + invSpeed)/2);
    float power = PID.Compute(PID.Target - avgSpeed);


    if (Controller.get_digital(CONVEYOR_IN))
    {
        Handle(NormalGroup.move(power));
        //Handle(InvertedGroup.move(-power));
    } else if (Controller.get_digital(CONVEYOR_OUT))
    {
        Handle(NormalGroup.move(-power));
        //Handle(InvertedGroup.move(power));
    } else
    {
        Handle(NormalGroup.brake());
        //Handle(InvertedGroup.brake());
    }

    if (Controller.get_digital(SCORE_TOP))
    {
        Handle(InvertedGroup.move(power));
        //Handle(InvertedGroup.move(-power));
    } else if (Controller.get_digital(SCORE_BOTTOM))
    {
        Handle(InvertedGroup.move(-power));
        //Handle(InvertedGroup.move(power));
    } else
    {
        Handle(InvertedGroup.brake());
        //Handle(InvertedGroup.brake());
    }

    // mogo
    if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    {
        Inverted = !Inverted;
        Handle(Splitter.set_value(Inverted));
    }
}

#endif //CONVEYOR_H
