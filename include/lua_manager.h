//
// Created by aaron on 8/16/2025.
//

#ifndef LUA_MANAGER_H
#define LUA_MANAGER_H
#include <string>

#include "lua/lua.hpp"

class lua_manager
{
    lua_State *LuaState;
public:
    lua_manager();
    ~lua_manager();

    void RunFile(std::string file);

    static lua_State* GetLuaState();
    static lua_manager* Get();
};

/// Lua function documentation is representative of the lua function.
namespace lua_functions
{
    void register_functions();

    /// Moves the conveyor at full power for x milliseconds
    /// @param x How long to move the conveyor, in milliseconds
    /// @return None.
    int l_MoveConveyorFor(lua_State* L);
}

#endif //LUA_MANAGER_H
