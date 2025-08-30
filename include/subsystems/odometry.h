//
// Created by aaron on 7/23/2025.
//

#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "../cls/subsystem.h"
#include "../lemlib/chassis/chassis.hpp"

class odometry final : public subsystem
{
    friend class std::unique_ptr<odometry>;
public:
    pros::Imu Inertial;
    pros::Rotation RotationVertical;
    pros::Rotation RotationHorizontal;

    lemlib::TrackingWheel TrackingHorizontal;
    lemlib::TrackingWheel TrackingVertical;
    lemlib::OdomSensors OdometrySensors;

public:
    odometry();

protected:
    //Called in drivetrain tick.
    void tick_implementation() override;

public:
    static odometry* Get();
};

#endif //ODOMETRY_H
