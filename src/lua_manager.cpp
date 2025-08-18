//
// Created by aaron on 8/16/2025.
//
#include "../include/lua_manager.h"
#include "../include/conveyor.h"
#include "../include/lemlib/asset.hpp"
#include <memory>

//Define asset files
ASSET(testing_lua);

#define MAKE_STRING(x) #x

std::unique_ptr<lua_manager> manager = nullptr;
//This is done so that the deconstructor will automatically be called when the program exits.

lua_manager::lua_manager()
{
    LuaState = luaL_newstate();
    luaL_openlibs(LuaState);
}

lua_manager::~lua_manager()
{
    lua_close(LuaState);
}

void lua_manager::RunFile(std::string file)
{

    const char* script = nullptr;
    if (file == MAKE_STRING(testing_lua))
    {
        script = (const char*)testing_lua;
    } else if (file == MAKE_STRING(other_asset))
    {

    }

    if (script == nullptr) return;

    luaL_dostring(GetLuaState(), script);
}

lua_State *lua_manager::GetLuaState()
{
    return Get()->LuaState;
}

lua_manager *lua_manager::Get()
{
    if (!manager) manager = std::make_unique<lua_manager>();
    return manager.get();
}

void lua_functions::register_functions()
{
    lua_register(lua_manager::GetLuaState(), "MoveConveyorFor", l_MoveConveyorFor);
}

int lua_functions::l_MoveConveyorFor(lua_State *L)
{
    int miliseconds = luaL_checkinteger(L, 1);
    Handle(conveyor::Get()->ConveyorGroup.move(127));
    pros::delay(miliseconds);
    Handle(conveyor::Get()->ConveyorGroup.brake());
    return 0;
}



