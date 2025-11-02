//
// Created by aaron on 8/25/2025.
//

#include "../../include/cls/subsystem.hpp"

subsystem::subsystem(bool start_active)
{
    b_is_active = start_active;
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
    bool res = activate_implementation();
    b_is_active = true;
    return res;
}

bool subsystem::deactivate()
{
    bool res = deactivate_implementation();
    b_is_active = false;
    return res;
}

void subsystem::tick()
{
    if (is_active()) tick_implementation();
}