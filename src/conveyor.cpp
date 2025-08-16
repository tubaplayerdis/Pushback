#include "../include/conveyor.h"

#define FULL_POWER 127

//Private Singleton
conveyor* instance;

void conveyor::Tick_Implementation()
{
    if (Controller.get_digital(CONVEYOR_IN))
    {
        Handle(ConveyorGroup.move(FULL_POWER));
    } else if (Controller.get_digital(CONVEYOR_OUT))
    {
        Handle(ConveyorGroup.move(-FULL_POWER));
    } else
    {
        Handle(ConveyorGroup.brake());
    }
    //Add color sensing logic etc
}

conveyor *conveyor::Get()
{
    if (!instance) instance = new conveyor();
    return instance;
}
