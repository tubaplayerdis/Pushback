#include "../../include/subsystems/conveyor.h"

#define FULL_POWER 127

//Private Singleton
std::unique_ptr<conveyor> conveyor_instance;

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
