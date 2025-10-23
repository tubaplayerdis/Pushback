//
// Created by aaron on 7/24/2025.
//

#ifndef AUTONS_H
#define AUTONS_H

#include "titanselect/titanselect.hpp"

//Macro expands the pos struct to use with lemlib commands
#define POS(pos) pos.X, pos.Y, pos.T

struct pos
{
    const float X;
    const float Y;
    const float T;

    pos(const float x, const float y, const float t) : X(x), Y(y), T(t) {}
};

namespace autons
{
    extern ts::auton testing;
    extern ts::auton nine_left;
    extern ts::auton nine_right;
    extern ts::auton nine_awp_low;
    extern ts::auton nine_awp_high;
    extern ts::auton skills;
}


#endif //AUTONS_H
