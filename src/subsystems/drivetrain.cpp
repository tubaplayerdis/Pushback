//
// Created by aaron on 8/22/2025.
//
#include "../../include/subsystems/drivetrain.hpp"
#include "../../include/ports.hpp"

std::unique_ptr<drivetrain> drivetrain_instance;

using namespace ports::drivetrain;
using namespace ports::drivetrain::settings;

drivetrain::drivetrain() :
motors_left({LEFT_A, LEFT_B, LEFT_C}, DRIVETRAIN_MOTOR_CARTRIDGE),
motors_right({RIGHT_A, RIGHT_B, RIGHT_C}, DRIVETRAIN_MOTOR_CARTRIDGE)
{

}

void drivetrain::tick_implementation()
{
    constexpr auto FULL_POWER = 127;

    if (controller_master.get_digital(ports::drivetrain::controls::SWING_LEFT))
    {
        (void)motors_left.set_brake_mode_all(pros::MotorBrake::hold);
        (void)motors_right.move(FULL_POWER);
    }
    else if (controller_master.get_digital(ports::drivetrain::controls::SWING_RIGHT))
    {
        (void)motors_left.move(FULL_POWER);
        (void)motors_right.set_brake_mode_all(pros::MotorBrake::hold);
    }
    else
    {
        if (motors_left.get_brake_mode(0) != pros::MotorBrake::coast || motors_right.get_brake_mode(0) != pros::MotorBrake::coast)
        {
            (void)motors_left.set_brake_mode_all(pros::MotorBrake::coast);
            (void)motors_right.set_brake_mode_all(pros::MotorBrake::coast);
        }
    }
}

drivetrain* drivetrain::get()
{
    if (!drivetrain_instance) drivetrain_instance = std::unique_ptr<drivetrain>(new drivetrain());
    return drivetrain_instance.get();
}