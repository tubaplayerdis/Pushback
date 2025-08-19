//
// Created by aaron on 8/16/2025.
//

#ifndef LUA_MANAGER_H
#define LUA_MANAGER_H
#include <string>
#include <vector>

#include "lemlib/asset.hpp"
#include "lua/lua.hpp"

#define LUA_ASSET(x) \
    ASSET(x); \
    static asset_wrapper wrapper_##x = asset_wrapper(x, #x);

struct asset_wrapper;

class lua_manager
{
    static std::vector<asset_wrapper*> assets;
    lua_State *LuaState;
public:
    static void RegisterAsset(asset_wrapper *asset)
    {
        assets.push_back(asset);
    }

    lua_manager();
    ~lua_manager();

    void RunFile(std::string file);

    static lua_State* GetLuaState();
    static lua_manager* Get();
};

struct asset_wrapper
{
    asset ref;
    const char* name;
    explicit asset_wrapper(asset asset_ptr, const char* _name) : ref(asset_ptr), name(_name)
    {
        lua_manager::RegisterAsset(this);
    }
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
