//
// Created by aaron on 7/20/2025.
//

#ifndef ROBOT_H
#define ROBOT_H
#include "conveyor.h"
#include "lemlib//chassis/chassis.hpp"
#include "pros/motor_group.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

//Electronics are organized under robot then its according subsystem then its according type
namespace robot
{
    bool CalibrateRobot();
    void InitializeTasks();
    void TickSubsystems();

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

    lemlib::Chassis Chassis(drivetrain::Drivetrain, controller::ControllerLateral, controller::ControllerAngular, drivetrain::odometry::Odometry, &controller::CurveThrottle, &controller::CurveSteer);
}

#endif //ROBOT_H
