//
// Created by aaron on 7/23/2025.
//

#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "../cls/subsystem.h"
#include "../lemlib/chassis/chassis.hpp"

constexpr auto PORT_INERTIAL = 14;
constexpr auto PORT_ROTATION_VERTICAL = -13;
constexpr auto PORT_ROTATION_HORIZONTAL = -8;

constexpr auto ODOMETRY_DIST_FROM_CENTER_HORIZONTAL = -1.5;
constexpr auto ODOMETRY_DIST_FROM_CENTER_VERTICAL = 0;
constexpr auto ODOMETRY_WHEEL_SIZE = lemlib::Omniwheel::NEW_2;

class odometry final : public subsystem
{
public:
    pros::Imu Inertial;
    pros::Rotation RotationVertical;
    pros::Rotation RotationHorizontal;

    lemlib::TrackingWheel TrackingHorizontal;
    lemlib::TrackingWheel TrackingVertical;
    lemlib::OdomSensors OdometrySensors;

public:
    odometry() :
    Inertial(PORT_INERTIAL), RotationVertical(PORT_ROTATION_VERTICAL),
    RotationHorizontal(PORT_ROTATION_HORIZONTAL),
    TrackingHorizontal(&RotationHorizontal, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_HORIZONTAL),
    TrackingVertical(&RotationVertical, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_VERTICAL),
    OdometrySensors(&TrackingVertical, nullptr, &TrackingHorizontal, nullptr, &Inertial) {}

protected:
    //Called in drivetrain tick.
    void Tick_Implementation() override;

public:
    static odometry* Get();
};

#endif //ODOMETRY_H
