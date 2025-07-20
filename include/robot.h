//
// Created by aaron on 7/20/2025.
//

#ifndef ROBOT_H
#define ROBOT_H
#include "lemlib//chassis/chassis.hpp"
#include "pros/motor_group.hpp"

//Electronics are organized under robot then its according subsystem then its according type
namespace robot
{

    namespace drivetrain
    {
        namespace motors
        {
            pros::MotorGroup LeftGroup({-1,-1,-1});
            pros::MotorGroup RightGroup({-1,-1,-1});
        }

        lemlib::Drivetrain Drivetrain(&motors::LeftGroup, &motors::RightGroup, 10, lemlib::Omniwheel::NEW_325, 450, 2);

        //TODO: setup drivetrain curves and lemlib "chassis"
    }

    namespace conveyor
    {
        namespace motors
        {
            pros::MotorGroup NormalGroup({-1,-1}); //Runs the intake and other system requiring the path of movement.
            pros::MotorGroup InvertedGroup({-1,-1}); //Moslty resposible for getting the blocks into storage
        }

        namespace pneumatics
        {
            pros::adi::Pneumatics Splitter(-1, false); //Dictates whether balls are stowed/scored.
        }
    }
}

#endif //ROBOT_H
