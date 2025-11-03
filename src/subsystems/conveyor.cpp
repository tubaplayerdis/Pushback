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
intake (INTAKE),
exhaust(EXHAUST),
conveyor_group({CONVEYOR_A, CONVEYOR_B}),
splitter_optical(SPLITTER_OPTICAL),
ramp(RAMP, false),
lift(LIFT, false),
wings(WINGS, false),
color_sort_color(object_color::NEUTRAL)
{
    splitter_optical.set_led_pwm(SPLITTER_BRIGHTNESS); //50% brightness
    exhaust.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

/// Whether x is in the specified range. (inclusive)
/// \param x input number
/// \param low lower limit
/// \param high upper limit
/// \return boolean representative of whether x was in the specified range.
static bool range(double x, double low, double high)
{
    return x >= low && x <= high;
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

void conveyor::do_color_sort(bool* out_did_color_sort)
{
    //If an object is not detected, keep the current piston configuration and continue the loop.
    if (splitter_optical.get_proximity() == 255)
    {
        if (ramp.is_extended()) ramp.retract();
    }

    //If the detected color is blue and the chosen color is red, extend the piston
    if ((detect_color(&splitter_optical) == BLUE && color_sort_color == RED) || (detect_color(&splitter_optical) == RED && color_sort_color == BLUE))
    {
        ramp.extend(); //Reversed due to setup IRL
        *out_did_color_sort = true;
    }
    else if (detect_color(&splitter_optical) == color_sort_color) //If the detected color and chosen color match retract the piston.
    {
        ramp.retract(); //Reversed due to setup IRL
        *out_did_color_sort = true;
    }
}

bool conveyor::is_color_sort_active() {
    return color_sort_color != NEUTRAL;
}

bool conveyor::toggle_color_sort() {
    switch (color_sort_color)
    {
        case NEUTRAL:
            color_sort_color = BLUE;
            break;
        case BLUE:
            color_sort_color = RED;
            break;
        case RED:
            color_sort_color = NEUTRAL;
            break;
    }
    return is_color_sort_active();
}

void conveyor::tick_implementation() {

    const char* color_display_code = "NONE ";
    switch (color_sort_color)
    {
        case NEUTRAL:
            color_display_code = "NONE ";
            break;
        case BLUE:
            color_display_code = "RED  ";
            break;
        case RED:
            color_display_code = "BLUE ";
            break;
    }
    //controller_master.print(1,0,"Excluding Color: %s", color_display_code);

    if (controller_master.get_digital(RAMP_MACRO))
    {
        (void)exhaust.move(FULL_POWER);
        (void)conveyor_group.move(FULL_POWER);
        (void)intake.move(FULL_POWER);
        (void)ramp.extend();
    } else
    {
        bool did_color_sort = false;
        bool did_exhaust = false;
        if (controller_master.get_digital(EXHAUST_OUT))
        {
            (void)exhaust.move(FULL_POWER);

            if (is_color_sort_active())
            {
                do_color_sort(&did_color_sort);
            }

            did_exhaust = true;
        }

        if (controller_master.get_digital(CONVEYOR_IN))
        {
            if (ramp.is_extended() && !did_color_sort) ramp.retract(); //Color sort will do this
            (void)conveyor_group.move(FULL_POWER);
            (void)intake.move(FULL_POWER);
            if (!did_exhaust) (void)exhaust.move(-0.1 * FULL_POWER);
        } else if (controller_master.get_digital(CONVEYOR_OUT))
        {
            if (ramp.is_extended()) ramp.retract();
            (void)exhaust.move(-FULL_POWER);
            (void)conveyor_group.move(-FULL_POWER);
            (void)intake.move(-FULL_POWER);
        } else
        {
            (void)conveyor_group.brake();
            (void)intake.brake();
            if(!did_exhaust) (void)exhaust.brake();
        }


    }

    if (controller_master.get_digital_new_press(TOGGLE_LIFT))
    {
        (void)lift.toggle();
    }

    if (controller_master.get_digital_new_press(TOGGLE_WINGS))
    {
        // Toggle the ears (wings) that can fit into the long goals.
        (void)wings.toggle();
    }

    if (controller_master.get_digital_new_press(OVERRIDE_RAMP_UP))
    {
        (void)ramp.extend();
    }

    if (controller_master.get_digital_new_press(OVERRIDE_RAMP_DOWN))
    {
        (void)ramp.retract();
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
