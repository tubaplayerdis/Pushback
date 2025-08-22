//
// Created by aaron on 8/22/2025.
//
#include "../include/drivetrain.h"
#include "../include/cls/subsystem.h"
#include "../include/pros/adi.hpp"
#include "../include/pros/motor_group.hpp"
#include "../include/pros/misc.h"
#include "../include/controller.h"
#include "../include/monitor.h"
#include "../include/odometry.h"

std::unique_ptr<drivetrain> instance;

void drivetrain::Tick_Implementation()
{
    if (!IsActive()) return;
    ODOMETRY->Tick();
    const int32_t throttle = Controller.get_analog(CONTROLLER_VERTICAL_AXIS);
    const int32_t turn = -1 * Controller.get_analog(CONTROLLER_HORIZONTAL_AXIS);
    Chassis.arcade(throttle, turn);
    //Handle moving the drivetrain.
}

drivetrain* drivetrain::Get()
{
    if (!instance) instance = std::make_unique<drivetrain>();
    return instance.get();
}