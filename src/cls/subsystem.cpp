//
// Created by aaron on 8/25/2025.
//

#include "../../include/cls/subsystem.h"

subsystem::subsystem(bool bStartActive)
{
    b_is_active = bStartActive;
}

bool subsystem::is_active()
{
    return b_is_active;
}

bool subsystem::activate_implementation()
{
    return true;
}

bool subsystem::deactivate_implementation()
{
    return true;
}

bool subsystem::activate()
{
    bool bResult = activate_implementation();
    b_is_active = true;
    return bResult;
}

bool subsystem::deactivate()
{
    bool bResult = deactivate_implementation();
    b_is_active = false;
    return bResult;
}

void subsystem::tick()
{
    if (is_active()) tick_implementation();
}