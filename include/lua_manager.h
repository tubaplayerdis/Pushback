//
// Created by aaron on 8/16/2025.
//

#ifndef LUA_MANAGER_H
#define LUA_MANAGER_H
#include <string>
#include <vector>

#include "lemlib/asset.hpp"
#include "lua/lua.hpp"

//Used for lua files. use an _ when a dot is present in the file name.
#define LUA_ASSET(x) \
    ASSET(x); \
    static asset_wrapper wrapper_##x = asset_wrapper(x, #x);

//Define lua compatible functions. lambda does not have to be a lambda it can be a function pointer that matches the int(*)(lua_State *);
#define LUA_FUNCTION(name, lambda) \
    static lua_function name(#name , lambda);

struct lua_function;
struct asset_wrapper;

class lua_manager
{
    static std::vector<asset_wrapper*> assets;
    static std::vector<lua_function*> function_pointers;
    lua_State *LuaState;

public:

    static void RegisterFunction(lua_function* pointer)
    {
        function_pointers.push_back(pointer);
    }

    static void RegisterAsset(asset_wrapper *asset)
    {
        assets.push_back(asset);
    }

    lua_manager();
    ~lua_manager();

    void RegisterFunctions();
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

struct lua_function
{
    int(*function_pointer)(lua_State *);
    const char* name;

    explicit lua_function(const char* name, int(*_pointer)(lua_State *)) : name(name), function_pointer(_pointer)
    {
        lua_manager::RegisterFunction(this);
    }
};
#endif //LUA_MANAGER_H
