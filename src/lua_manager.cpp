//
// Created by aaron on 8/16/2025.
//
#include "../include/lua_manager.h"
#include "../include/conveyor.h"
#include "../include/lemlib/asset.hpp"
#include <vector>
#include <memory>

std::unique_ptr<lua_manager> manager = nullptr;
std::vector<asset_wrapper*> lua_manager::assets = std::vector<asset_wrapper*>();
//This is done so that the deconstructor will automatically be called when the program exits.

//Define LUA assets, ie files.
LUA_ASSET(testing_lua)

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
    for (asset_wrapper* wrappers : assets)
    {
        if (file != wrappers->name) continue;
        luaL_dostring(LuaState, (const char*)wrappers->ref.buf);
    }

    //Handle a file not being ran
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



