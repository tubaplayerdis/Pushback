//
// Created by aaron on 7/23/2025.
//

#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "cls/subsystem.h"
#include "lemlib/chassis/chassis.hpp"

#define PORT_INERTIAL 14
#define PORT_ROTATION_VERTICAL -13
#define PORT_ROTATION_HORIZONTAL -8

#define ODOMETRY_DIST_FROM_CENTER_HORIZONTAL -1.5
#define ODOMETRY_DIST_FROM_CENTER_VERTICAL 0
#define ODOMETRY_WHEEL_SIZE lemlib::Omniwheel::NEW_2

#define ODOMETRY odometry::Get()

class odometry final : public subsystem
{
    inline static odometry* instance;
public:
    pros::Imu Inertial;
    pros::Rotation RotationVertical;
    pros::Rotation RotationHorizontal;

    lemlib::TrackingWheel TrackingHorizontal;
    lemlib::TrackingWheel TrackingVertical;
    lemlib::OdomSensors OdometrySensors;

private:
    odometry() :
    Inertial(PORT_INERTIAL), RotationVertical(PORT_ROTATION_VERTICAL),
    RotationHorizontal(PORT_ROTATION_HORIZONTAL),
    TrackingHorizontal(&RotationHorizontal, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_HORIZONTAL),
    TrackingVertical(&RotationVertical, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_VERTICAL),
    OdometrySensors(&TrackingVertical, nullptr, &TrackingHorizontal, nullptr, &Inertial) {}

protected:
    void Tick_Implementation() override;

public:
    static odometry* Get();
};

inline odometry* odometry::Get()
{
    if (!instance) instance = new odometry();
    return instance;
}

inline void odometry::Tick_Implementation()
{
    //Add debug statements or other things
}

#endif //ODOMETRY_H
