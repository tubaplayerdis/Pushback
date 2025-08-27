//
// Created by aaron on 8/25/2025.
//
#include "../../include/subsystems/odometry.h"
#include <memory>


std::unique_ptr<odometry> odometry_instance;

odometry* odometry::Get()
{
    if (!odometry_instance) odometry_instance = std::make_unique<odometry>();
    return odometry_instance.get();
}

void odometry::Tick_Implementation()
{
    //Add debug statements or other things
}