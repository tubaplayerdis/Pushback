//
// Created by aaron on 7/20/2025.
//

#ifndef ROBOT_H
#define ROBOT_H
#include "lemlib//chassis/chassis.hpp"
#include "pros/motor_group.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

//Define control macros
#define CONVEYOR_IN pros::E_CONTROLLER_DIGITAL_L1
#define CONVEYOR_OUT pros::E_CONTROLLER_DIGITAL_R1

#define FULL_POWER_FWD 127

//Electronics are organized under robot then its according subsystem then its according type
namespace robot
{
    bool calibrateRobot();
    void initializeTasks();

    //Handle various subsystems and controls/monitors
    namespace tasks
    {
        pros::Task* ControllerGraphics();
        pros::Task* Monitor();
    }

    namespace drivetrain
    {
        //Calculates and sends out drivetrain controls
        void drivetrainControl();

        namespace odometry
        {
            namespace sensors
            {
                pros::Imu Inertial(14);

                pros::Rotation RotationVertical(-13);

                pros::Rotation RotationHorizontial(-8);
            }

            // horizontal tracking wheel
            lemlib::TrackingWheel TrackingHorizontial(&sensors::RotationHorizontial, lemlib::Omniwheel::NEW_2, -1.5);

            // vertical tracking wheel
            lemlib::TrackingWheel TrackingVertical(&sensors::RotationVertical, lemlib::Omniwheel::NEW_2, 0);

            // odometry settings
            lemlib::OdomSensors Odometry(&TrackingVertical, nullptr, &TrackingHorizontial, nullptr, &sensors::Inertial);
        }

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
        void conveyorControl();

        inline bool isConveyorActive = false;

        namespace motors
        {
            pros::MotorGroup NormalGroup({-1,-1}); //Runs the intake and other system requiring the path of movement.
            pros::MotorGroup InvertedGroup({-1,-1}); //Mostly resposible for getting the blocks into storage
        }

        namespace pneumatics
        {
            pros::adi::Pneumatics Splitter(-1, false); //Dictates whether balls are stowed/scored.
        }
    }

    namespace controller
    {
        lemlib::ControllerSettings
        ControllerLateral(8,   // proportional gain (kP)
                           0,   // integral gain (kI)
                           4,   // derivative gain (kD)
                           3,   // anti windup
                           1,   // small error range, in inches
                           100, // small error range timeout, in milliseconds
                           3,   // large error range, in inches
                           500, // large error range timeout, in milliseconds
                           20   // maximum acceleration (slew)
        );

        // angular PID controller
        lemlib::ControllerSettings
        ControllerAngular(2,   // proportional gain (kP)
                           0,   // integral gain (kI)
                           10,  // derivative gain (kD)
                           3,   // anti windup
                           1,   // small error range, in degrees
                           100, // small error range timeout, in milliseconds
                           3,   // large error range, in degrees
                           500, // large error range timeout, in milliseconds
                           0    // maximum acceleration (slew)
        );

        // input curve for throttle input during driver control
        lemlib::ExpoDriveCurve
        CurveThrottle(3,    // joystick deadband out of 127
                       10,   // minimum output where drivetrain will move out of 127
                       1.019 // expo curve gain
        );

        // input curve for steer input during driver control
        lemlib::ExpoDriveCurve
        CurveSteer(3,    // joystick deadband out of 127
                    10,   // minimum output where drivetrain will move out of 127
                    1.035 // expo curve gain
        );
    }

    pros::Controller Controller(pros::E_CONTROLLER_MASTER);

    lemlib::Chassis Chassis(drivetrain::Drivetrain, controller::ControllerLateral, controller::ControllerAngular, drivetrain::odometry::Odometry, &controller::CurveThrottle, &controller::CurveSteer);
}

#endif //ROBOT_H
