#include "../../include/subsystems/conveyor.hpp"
#include "../../include/ports.hpp"
#include <memory>
#include "../../include/subsystems/localization.hpp"

constexpr auto FULL_POWER = 127;

//Private Singleton
std::unique_ptr<conveyor> conveyor_instance;

constexpr auto RED_LOW = 0;
constexpr auto RED_HIGH = 15;
constexpr auto BLUE_LOW = 100;
constexpr auto BLUE_HIGH = 250;
constexpr auto HIGH_TROUGH_LOW = 100;
constexpr auto HIGH_TROUGH_HIGH = 100;
constexpr auto LOW_TROUGH_LOW = 100;
constexpr auto LOW_TROUGH_HIGH = 100;

using namespace ports::conveyor;
using namespace ports::conveyor::controls;

conveyor::conveyor() :
        subsystem(),
        exhaust(EXHAUST),
        conveyor_intake(CONVEYOR),
        trapdoor(TRAPDOOR, false),
        match_loader(MATCH_LOADER, false),
        wings(WINGS, false)
{
    (void)exhaust.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void conveyor::tick_implementation() {

    if (controller_master.get_digital_new_press(RAMP_MACRO))
    {
        pros::Task macro_task([this]()
        {
            (void)conveyor_intake.move(-FULL_POWER);
            pros::Task::delay(250);
            while (controller_master.get_digital(RAMP_MACRO))
            {
                (void)exhaust.move(-FULL_POWER * 0.75);
                (void)conveyor_intake.move(FULL_POWER * 0.5);
                (void)trapdoor.extend();
            }
        });
        (void)macro_task.get_priority();
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
            if (!did_exhaust) (void)exhaust.move(-0.1 * FULL_POWER);
        } else if (controller_master.get_digital(CONVEYOR_OUT))
        {
            if (trapdoor.is_extended()) trapdoor.retract();
            (void)exhaust.move(-FULL_POWER);
            (void)conveyor_intake.move(-FULL_POWER);
        } else
        {
            (void)conveyor_intake.brake();
            if(!did_exhaust) (void)exhaust.brake();
        }
    }

    if (controller_master.get_digital_new_press(TOGGLE_MATCH_LOADER))
    {
        (void)match_loader.toggle();
    }

    if (controller_master.get_digital_new_press(TOGGLE_WINGS))
    {
        // Toggle the ears (wings) that can fit into the long goals.
        (void)wings.toggle();
    }
}

conveyor *conveyor::get()
{
    if (!conveyor_instance) conveyor_instance = std::unique_ptr<conveyor>( new conveyor() );
    return conveyor_instance.get();
}
