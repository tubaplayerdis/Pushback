//
// Created by aaron on 8/25/2025.
//

#include "../../include/cls/subsystem.h"

subsystem::subsystem(bool bNeedsInit, bool bStartActive)
{
    bNeedsInit ? Initialization = EInitializationState::UNINITIALIZED : Initialization = EInitializationState::NONAPPLICABLE;
    isActive = bStartActive;
}

bool subsystem::IsActive()
{
    return isActive;
}

EInitializationState subsystem::GetInitializationState()
{
    return Initialization;
}

bool subsystem::Activate_Implementation()
{
    return true;
}

bool subsystem::Deactivate_Implementation()
{
    return true;
}

bool subsystem::Activate()
{
    bool bResult = Activate_Implementation();
    isActive = true;
    return bResult;
}

bool subsystem::Deactivate()
{
    bool bResult = Deactivate_Implementation();
    isActive = false;
    return bResult;
}

void subsystem::Tick()
{
    if (IsActive()) Tick_Implementation();
}