//
// Created by aaron on 8/19/2025.
//

#ifndef LUA_FUNCTIONS_H
#define LUA_FUNCTIONS_H

#include "lua_manager.h"
#include "conveyor.h"
#include "drivetrain.h"

LUA_FUNCTION(MoveConveyorFor, [](lua_State* L) -> int
{
    int miliseconds = luaL_checkinteger(L, 1);
    Handle(conveyor::Get()->ConveyorGroup.move(127));
    pros::delay(miliseconds);
    Handle(conveyor::Get()->ConveyorGroup.brake());
    return 0;
});

LUA_FUNCTION(TurnTo, [](lua_State* L) -> int
{
    double heading = luaL_checknumber(L, 1);
    drivetrain::Get()->Chassis.turnToHeading(heading, 1000);
    return 0;
});

#endif //LUA_FUNCTIONS_H
