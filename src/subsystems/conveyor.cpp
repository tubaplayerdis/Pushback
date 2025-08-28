#include "../../include/subsystems/conveyor.h"

constexpr auto FULL_POWER = 127;

//Private Singleton
std::unique_ptr<conveyor> conveyor_instance;

#define range(x, low, high) x < low && x > high

constexpr auto RED_LOW = 0;
constexpr auto RED_HIGH = 15;
constexpr auto BLUE_LOW = 150;
constexpr auto BLUE_HIGH = 250;

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
    pros::Optical* optical = &conveyor::Get()->SplitterOptical;
    pros::adi::Pneumatics* splitter = &conveyor::Get()->Splitter;
    ColorType color = conveyor::Get()->ColorSortStatus;

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
    if (!ColorSortTask) ColorSortTask = std::make_unique<pros::Task>(color_sort_loop);
}

void conveyor::DeactivateColorSort()
{
    if (!ColorSortTask) return;
    ColorSortTask->remove();
    ColorSortTask.reset();
}

void conveyor::Tick_Implementation()
{
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
    //Add color sensing logic etc
}

conveyor *conveyor::Get()
{
    if (!conveyor_instance) conveyor_instance = std::make_unique<conveyor>();
    return conveyor_instance.get();
}
