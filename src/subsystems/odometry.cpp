//
// Created by aaron on 8/25/2025.
//
#include "../../include/subsystems/odometry.h"
#include "../../include/ports.h"
#include <memory>


std::unique_ptr<odometry> odometry_instance;

using namespace ports::odometry;
using namespace ports::odometry::settings;

odometry::odometry() :
Inertial(INERTIAL), RotationVertical(ROTATION_VERTICAL),
RotationHorizontal(ROTATION_HORIZONTAL),
TrackingHorizontal(&RotationHorizontal, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_HORIZONTAL),
TrackingVertical(&RotationVertical, ODOMETRY_WHEEL_SIZE, ODOMETRY_DIST_FROM_CENTER_VERTICAL),
OdometrySensors(&TrackingVertical, nullptr, &TrackingHorizontal, nullptr, &Inertial)
{

}

odometry* odometry::Get()
{
    if (!odometry_instance) odometry_instance = std::unique_ptr<odometry>( new odometry() );
    return odometry_instance.get();
}

void odometry::tick_implementation() {
    //Add debug statements or other things
}