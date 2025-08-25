//
// Created by aaron on 8/25/2025.
//
#include "../../include/subsystems/odometry.h"


std::unique_ptr<odometry> instance;

odometry* odometry::Get()
{
    if (!instance) instance = std::make_unique<odometry>();
    return instance.get();
}

void odometry::Tick_Implementation()
{
    //Add debug statements or other things
}