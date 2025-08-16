//
// Created by aaron on 8/16/2025.
//

#ifndef LUA_MANAGER_H
#define LUA_MANAGER_H
#include "lua/lua.hpp"

class lua_manager
{
    lua_State *LuaState;
public:
    lua_manager();
    ~lua_manager();

    static lua_State* GetLuaState();
    static lua_manager* Get();
};



#endif //LUA_MANAGER_H
