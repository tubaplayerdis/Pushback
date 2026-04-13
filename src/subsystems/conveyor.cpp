#include "../../include/subsystems/conveyor.hpp"
#include "../../include/ports.hpp"
#include <memory>

#include "../../include/pros/rtos.h"
#include "../../include/subsystems/localization.hpp"

constexpr auto FULL_POWER = 127;
constexpr auto EXHAUST_INDEX = -0.15 * FULL_POWER;

//Private Singleton
std::unique_ptr<conveyor> conveyor_instance;

using namespace ports::conveyor;
using namespace ports::conveyor::controls;

double get_watts(pros::Motor &m) {
    // Voltage is in mV, Current is in mA
    double voltage = m.get_voltage() / 1000.0; // Convert to Volts
    double current = m.get_current_draw() / 1000.0; // Convert to Amps
    return voltage * current;
}

conveyor::conveyor() :
        subsystem(),
        exhaust(EXHAUST),
        conveyor_intake(CONVEYOR),
        trapdoor(TRAPDOOR, true),
        low_goal(LOW_GOAL, false, true),
        match_loader(MATCH_LOADER, false),
        wings(WINGS, false),
        descore(DESCORE, false),
        do_low_goal_macro(false),
        low_goal_macro([this]() -> void
        {
            constexpr auto FRESH_BLOCK_OUTPUT = FULL_POWER * -0.375;
            constexpr auto OLD_BLOCK_OUTPUT = FULL_POWER * -0.475;

            while (true)
            {
                if (!do_low_goal_macro)
                {
                    pros::Task::delay(10);
                    continue;
                }

                /*
                 *"The motor firmware will cut maximum motor current in half at 55 degC, to 1/4 at 60 degC, 1/8 at 65 degC and disable the motor completely if it reaches 70 degC"
                 */

                //controller_master.print(1, 0, "T: %.2f        ", conveyor_intake.get_temperature());

                //Anti-jam
                if (get_watts(conveyor_intake) < -6.5)
                {
                    (void)conveyor_intake.move(FULL_POWER);
                    pros::Task::delay(200);

                    //Get back up to speed
                    (void)conveyor_intake.move(OLD_BLOCK_OUTPUT);
                    pros::Task::delay(300);
                }

                (void)conveyor_intake.move(OLD_BLOCK_OUTPUT);
                pros::Task::delay(10);
            }
        })
{
    (void)exhaust.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

bool do_brake = false;

void conveyor::tick_implementation() {

    if (controller_master.get_digital(RAMP_MACRO))
    {
        (void)exhaust.move(-FULL_POWER);
        (void)conveyor_intake.move(FULL_POWER * 0.7);
        (void)trapdoor.retract();
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
            if (!trapdoor.is_extended()) trapdoor.extend(); //Color sort will do this
            (void)conveyor_intake.move(FULL_POWER);
            if (!did_exhaust && do_brake)
            {
                (void)exhaust.brake();
                do_brake = false;
            }
        } else if (controller_master.get_digital(CONVEYOR_OUT))
        {
            if (!trapdoor.is_extended()) trapdoor.extend();
            (void)exhaust.move(-FULL_POWER);
            (void)conveyor_intake.move(-FULL_POWER);
            if (!do_brake) do_brake = true;
        } else if (controller_master.get_digital(ports::localization::controls::BARRIER_CROSS))
        {
            if (!trapdoor.is_extended()) trapdoor.extend();
            (void)conveyor_intake.move(FULL_POWER);
            if (!did_exhaust && do_brake)
            {
                (void)exhaust.brake();
                do_brake = false;
            }
        } else if (controller_master.get_digital(ports::conveyor::controls::HALF_OUT))
        {
            if (!trapdoor.is_extended()) trapdoor.extend();
            //conveyor_intake.move(FULL_POWER * -0.40);
            do_low_goal_macro = true;
        } else if (controller_master.get_digital(ports::conveyor::controls::QUARTER_OUT))
        {
            if (!trapdoor.is_extended()) trapdoor.extend();
            (void)conveyor_intake.move(FULL_POWER * -0.30);
            if (!did_exhaust && do_brake)
            {
                (void)exhaust.brake();
                do_brake = false;
            }
        }
        else
        {
            (void)conveyor_intake.brake();
            do_low_goal_macro = false;
            if (!did_exhaust)
            {
                (void)exhaust.brake();
            }
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
