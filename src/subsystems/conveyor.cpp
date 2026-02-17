#include "../../include/subsystems/conveyor.hpp"
#include "../../include/ports.hpp"
#include <memory>
#include "../../include/subsystems/localization.hpp"

constexpr auto FULL_POWER = 127;

//Private Singleton
std::unique_ptr<conveyor> conveyor_instance;

using namespace ports::conveyor;
using namespace ports::conveyor::controls;

conveyor::conveyor() :
        subsystem(),
        exhaust(EXHAUST),
        conveyor_intake(CONVEYOR),
        trapdoor(TRAPDOOR, false),
        match_loader(MATCH_LOADER, false),
        wings(WINGS, false),
        descore(DESCORE, false)
{
    (void)exhaust.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void conveyor::tick_implementation() {

    if (controller_master.get_digital(RAMP_MACRO))
    {
        (void)exhaust.move(-FULL_POWER * 0.8);
        (void)conveyor_intake.move(FULL_POWER * 0.65);
        (void)trapdoor.extend();
    } else
    {
        bool did_exhaust = false;
        if (controller_master.get_digital(EXHAUST_OUT))
        {
            (void)exhaust.move(FULL_POWER);
            did_exhaust = true;
        }

        if (controller_master.get_digital(CONVEYOR_IN))
        {
            if (trapdoor.is_extended()) trapdoor.retract(); //Color sort will do this
            (void)conveyor_intake.move(FULL_POWER);
            if (!did_exhaust) (void)exhaust.move(-0.2 * FULL_POWER);
        } else if (controller_master.get_digital(CONVEYOR_OUT))
        {
            if (trapdoor.is_extended()) trapdoor.retract();
            (void)exhaust.move(-FULL_POWER);
            (void)conveyor_intake.move(-FULL_POWER);
        } else if (controller_master.get_digital(ports::drivetrain::controls::BARRIER_CROSS))
        {
            if (trapdoor.is_extended()) trapdoor.retract();
            (void)conveyor_intake.move(FULL_POWER);
            (void)exhaust.move(FULL_POWER * -0.2);
        }
        else
        {
            (void)conveyor_intake.brake();
            if(!did_exhaust) (void)exhaust.brake();
        }
    }

    if (controller_master.get_digital_new_press(TOGGLE_MATCH_LOADER))
    {
        if (descore.is_extended() && !match_loader.is_extended()) descore.retract();
        (void)match_loader.toggle();
    }

    if (controller_master.get_digital_new_press(TOGGLE_DESCORE))
    {
        if (match_loader.is_extended() && !descore.is_extended()) match_loader.retract();
        (void)descore.toggle();
    }

    if (controller_master.get_digital(TOGGLE_WINGS))
    {
        wings.extend();
    }
    else
    {
        if (wings.is_extended())
        {
            wings.retract();
        }
    }
}

conveyor *conveyor::get()
{
    if (!conveyor_instance) conveyor_instance = std::unique_ptr<conveyor>( new conveyor() );
    return conveyor_instance.get();
}
