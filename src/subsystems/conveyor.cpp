#include "../../include/subsystems/conveyor.h"
#include "../../include/ports.h"
#include <memory>

constexpr auto FULL_POWER = 127;

//Private Singleton
std::unique_ptr<conveyor> conveyor_instance;

#define range(x, low, high) x < low && x > high

constexpr auto RED_LOW = 0;
constexpr auto RED_HIGH = 15;
constexpr auto BLUE_LOW = 150;
constexpr auto BLUE_HIGH = 250;

using namespace ports::conveyor;
using namespace ports::conveyor::controls;

conveyor::conveyor() :
subsystem(),
Intake (INTAKE),
Exhaust(EXHAUST),
SplitterOptical(SPLITTER_OPTICAL),
ConveyorGroup({CONVEYOR_A, CONVEYOR_B}),
Splitter(SPLITTER, false),
ColorSortTask(nullptr),
ColorSortStatus(ColorType::NEUTRAL)
{
    SplitterOptical.set_led_pwm(50); //50% brightness
}

/// Detects the color the spliter optical sensor sees
/// @return Enum representing the color the optical sensor sees.
static ColorType detect_color(pros::Optical* SplitterOptical)
{
    double hue = SplitterOptical->get_hue();

    if (range(hue, RED_LOW, RED_HIGH)) return RED;
    if (range(hue, BLUE_LOW, BLUE_HIGH)) return BLUE;

    return NEUTRAL;
}

static void color_sort_loop()
{
    pros::Optical* optical = &conveyor::get()->SplitterOptical;
    pros::adi::Pneumatics* splitter = &conveyor::get()->Splitter;
    ColorType color = conveyor::get()->ColorSortStatus;

    while (true)
    {
        pros::Task::delay(100);

        if (detect_color(optical) == BLUE && color == RED)
        {
            splitter->extend();
        }
        else if (detect_color(optical) == RED && color == BLUE)
        {
            splitter->extend();
        }
        else if (detect_color(optical) == color)
        {
            splitter->retract();
        }
    }
}

void conveyor::ActiveColorSort()
{
    if (!ColorSortTask) ColorSortTask = std::unique_ptr<pros::Task>( new pros::Task(color_sort_loop) );
}

void conveyor::DeactivateColorSort()
{
    if (!ColorSortTask) return;
    ColorSortTask->remove();
    ColorSortTask.reset();
}

void conveyor::tick_implementation() {
    if (Controller.get_digital(CONVEYOR_IN))
    {
        (void)ConveyorGroup.move(FULL_POWER);
        (void)Intake.move(FULL_POWER);
    } else if (Controller.get_digital(CONVEYOR_OUT))
    {
        (void)ConveyorGroup.move(-FULL_POWER);
        (void)Intake.move(-FULL_POWER);
    } else
    {
        (void)ConveyorGroup.brake();
        (void)Intake.brake();
    }

    if (Controller.get_digital(EXHAUST_OUT))
    {
        (void)Exhaust.move(FULL_POWER);
    } else (void)Exhaust.brake();

}

conveyor *conveyor::get()
{
    if (!conveyor_instance) conveyor_instance = std::unique_ptr<conveyor>( new conveyor() );
    return conveyor_instance.get();
}