#pragma once

#include <cstddef>
#include <cstdint>
#ifndef _ASSET_H_
#define _ASSET_H_

struct asset {
        uint8_t* buf;
        size_t size;
};

#define ASSET(x)                                                                                                                                                                                                          \
    extern uint8_t _binary_static_##x##_start[], _binary_static_##x##_size[];                                          \
    static asset x = {_binary_static_##x##_start, (size_t)_binary_static_##x##_size};                                  \

#define ASSET_LIB(x)                                                                                                   \
    extern "C" {                                                                                                       \
    extern uint8_t _binary_static_lib_##x##_start[], _binary_static_lib_##x##_size[];                                  \
    static asset x = {_binary_static_lib_##x##_start, (size_t)_binary_static_lib_##x##_size};                          \
    }

#endif // _ASSET_H_