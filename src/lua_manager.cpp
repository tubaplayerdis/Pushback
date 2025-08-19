//
// Created by aaron on 8/16/2025.
//
#include "../include/lua_manager.h"
#include "../include/conveyor.h"
#include "../include/lemlib/asset.hpp"
#include "../include/lua_functions.h"
#include <vector>
#include <memory>

#include "../include/drivetrain.h"

std::unique_ptr<lua_manager> manager = nullptr; //This is done so that the deconstructor will automatically be called when the program exits.
std::vector<asset_wrapper*> lua_manager::assets = std::vector<asset_wrapper*>();
std::vector<lua_function*> lua_manager::function_pointers = std::vector<lua_function*>();

//Define LUA assets, ie files.
LUA_ASSET(testing_lua)

lua_manager::lua_manager()
{
    LuaState = luaL_newstate();
    luaL_openlibs(LuaState);
    RegisterFunctions();
}

lua_manager::~lua_manager()
{
    lua_close(LuaState);
}

void lua_manager::RegisterFunctions()
{
    for (lua_function* f : function_pointers)
    {
        lua_register(GetLuaState(), f->name, f->function_pointer);
    }
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



