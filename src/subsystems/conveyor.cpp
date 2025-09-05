#include "../../include/subsystems/conveyor.h"
#include "../../include/ports.h"
#include <memory>

constexpr auto FULL_POWER = 127;

//Private Singleton
std::unique_ptr<conveyor> conveyor_instance;

constexpr auto RED_LOW = 0;
constexpr auto RED_HIGH = 15;
constexpr auto BLUE_LOW = 150;
constexpr auto BLUE_HIGH = 250;

using namespace ports::conveyor;
using namespace ports::conveyor::controls;

conveyor::conveyor() :
subsystem(),
intake (INTAKE),
exhaust(EXHAUST),
splitter_optical(SPLITTER_OPTICAL),
conveyor_group({CONVEYOR_A, CONVEYOR_B}),
splitter(SPLITTER, false),
lift(LIFT, false),
color_sort_task(nullptr),
color_sort_color(object_color::NEUTRAL),
color_sort_active(false)
{
    splitter_optical.set_led_pwm(50); //50% brightness
}

/// Whether x is in the specified range. (inclusive)
/// \param x input number
/// \param low lower limit
/// \param high upper limit
/// \return boolean representative of whether x was in the specified range.
static bool range(double x, double low, double high)
{
    return x <= low && x >= high;
}

/// Detects the color the spliter optical sensor sees
/// @return Enum representing the color the optical sensor sees.
static object_color detect_color(pros::Optical* splitter_optical)
{
    double hue = splitter_optical->get_hue();

    if (range(hue, RED_LOW, RED_HIGH)) return RED;
    if (range(hue, BLUE_LOW, BLUE_HIGH)) return BLUE;

    return NEUTRAL;
}

static void color_sort_loop()
{
    //Acquire pointers for efficiency
    pros::Optical* optical = &conveyor::get()->splitter_optical;
    pros::adi::Pneumatics* splitter = &conveyor::get()->splitter;
    object_color color = conveyor::get()->color_sort_color;

    while (true)
    {
        pros::Task::delay(100);

        //If an object is not detected, keep the current piston configuration and continue the loop.
        if (optical->get_proximity() == 255) continue;

        //If the detected color is blue and the chosen color is red, extend the piston
        if (detect_color(optical) == BLUE && color == RED)
        {
            splitter->extend();
        }
        else if (detect_color(optical) == RED && color == BLUE) //If the detected color is red and the chosen color is blue, extend the piston
        {
            splitter->extend();
        }
        else if (detect_color(optical) == color) //If the detected color and chosen color match retract the piston.
        {
            splitter->retract();
        }
    }
}

void conveyor::activate_color_sort()
{
    //If the unique_ptr holding the pros task is active, destroy the pros task and reset the pointer
    if(color_sort_task)
    {
        color_sort_task->remove();
        color_sort_task.reset();
    }

    //Reassign and restart the task. notify a status change by updating color_sort_active.
    color_sort_task = std::unique_ptr<pros::Task>( new pros::Task(color_sort_loop) );
    color_sort_active = true;
}

void conveyor::deactivate_color_sort()
{
    //If the color sort was already inactive, return early.
    if (!color_sort_task) return;

    //Destroy the task and reset the pointer
    color_sort_task->remove();
    color_sort_task.reset();

    //Notify that colorsort was turned off by updating color_sort_active
    color_sort_active = false;
}

bool conveyor::is_color_sort_active() {
    return color_sort_active;
}

bool conveyor::toggle_color_sort() {
    if (is_color_sort_active()) deactivate_color_sort();
    else activate_color_sort();
    return is_color_sort_active();
}

void conveyor::tick_implementation() {
    if (controller_master.get_digital(CONVEYOR_IN))
    {
        (void)conveyor_group.move(FULL_POWER);
        (void)intake.move(FULL_POWER);
    } else if (controller_master.get_digital(CONVEYOR_OUT))
    {
        (void)conveyor_group.move(-FULL_POWER);
        (void)intake.move(-FULL_POWER);
    } else
    {
        (void)conveyor_group.brake();
        (void)intake.brake();
    }

    if (controller_master.get_digital(EXHAUST_OUT))
    {
        (void)exhaust.move(FULL_POWER);
    } else (void)exhaust.brake();

    if (controller_master.get_digital_new_press(TOGGLE_LIFT))
    {
        (void)lift.toggle();
    }

    if (controller_master.get_digital_new_press(TOGGLE_COLOR_SORT))
    {
        toggle_color_sort();
    }

}

conveyor *conveyor::get()
{
    if (!conveyor_instance) conveyor_instance = std::unique_ptr<conveyor>( new conveyor() );
    return conveyor_instance.get();
}
