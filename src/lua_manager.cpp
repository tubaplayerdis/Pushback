//
// Created by aaron on 8/16/2025.
//
#include "../include/lua_manager.h"

#include <memory>

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

lua_State *lua_manager::GetLuaState()
{
    return Get()->LuaState;
}

lua_manager *lua_manager::Get()
{
    if (!manager) manager = std::make_unique<lua_manager>();
    return manager.get();
}



