#include "../../include/subsystems/conveyor.hpp"
#include "../../include/ports.hpp"
#include <memory>

#include "../../include/pros/rtos.h"
#include "../../include/subsystems/localization.hpp"

constexpr auto FULL_POWER = 127;
constexpr auto EXHAUST_INDEX = -0.25 * FULL_POWER;

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
        descore(DESCORE, false),
        do_low_goal_macro(false),
        low_goal_macro([this]() -> void
        {
            while (true)
            {
                if (!do_low_goal_macro)
                {
                    pros::Task::delay(50);
                    continue;
                }

                //Anti-jam
                if (conveyor_intake.get_efficiency() < 0.10)
                {
                    (void)conveyor_intake.move(FULL_POWER);
                    pros::Task::delay(100);
                }

                (void)conveyor_intake.move(FULL_POWER * -0.375);
                pros::Task::delay(50);
            }
        })
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
            if (!did_exhaust) (void)exhaust.move(EXHAUST_INDEX);
        } else if (controller_master.get_digital(CONVEYOR_OUT))
        {
            if (trapdoor.is_extended()) trapdoor.retract();
            (void)exhaust.move(-FULL_POWER);
            (void)conveyor_intake.move(-FULL_POWER);
        } else if (controller_master.get_digital(ports::localization::controls::BARRIER_CROSS))
        {
            if (trapdoor.is_extended()) trapdoor.retract();
            (void)conveyor_intake.move(FULL_POWER);
            (void)exhaust.move(EXHAUST_INDEX);
        } else if (controller_master.get_digital(ports::conveyor::controls::HALF_OUT))
        {
            if (trapdoor.is_extended()) trapdoor.retract();
            //conveyor_intake.move(FULL_POWER * -0.40);
            do_low_goal_macro = true;
        } else if (controller_master.get_digital(ports::conveyor::controls::QUARTER_OUT))
        {
            if (trapdoor.is_extended()) trapdoor.retract();
            (void)conveyor_intake.move(FULL_POWER * -0.30);
            (void)exhaust.move(EXHAUST_INDEX);
        }
        else
        {
            (void)conveyor_intake.brake();
            do_low_goal_macro = false;
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

    if (controller_master.get_digital_new_press(TOGGLE_WINGS))
    {
        wings.toggle();
    }
}

conveyor *conveyor::get()
{
    if (!conveyor_instance) conveyor_instance = std::unique_ptr<conveyor>( new conveyor() );
    return conveyor_instance.get();
}
