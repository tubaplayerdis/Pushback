#include "../../include/subsystems/conveyor.h"

#define FULL_POWER 127

//Private Singleton
std::unique_ptr<conveyor> conveyor_instance;

void conveyor::Tick_Implementation()
{
    if (Controller.get_digital(CONVEYOR_IN))
    {
        ConveyorGroup.move(FULL_POWER);
    } else if (Controller.get_digital(CONVEYOR_OUT))
    {
        ConveyorGroup.move(-FULL_POWER);
    } else
    {
        ConveyorGroup.brake();
    }

    if (Controller.get_digital())
    //Add color sensing logic etc
}

conveyor *conveyor::Get()
{
    if (!conveyor_instance) conveyor_instance = std::make_unique<conveyor>();
    return conveyor_instance.get();
}
