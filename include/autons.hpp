//
// Created by aaron on 7/24/2025.
//

#ifndef AUTONS_H
#define AUTONS_H

#include "titanselect/titanselect.hpp"

//Macro that expands the pos struct for use with the moveToPose command
#define POS(pos) pos.X, pos.Y, pos.T

//Macro that expands the pos struct for use with the moveToPoint command
#define MPOS(pos) pos.X, pos.Y

//Macro that expands the pos struct for use with the turnToHeading command
#define TPOS(pos) pos.T

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
    extern ts::auton nine_right_offset;
    extern ts::auton nine_awp_high;
    extern ts::auton nine_awp_high_offset;
    extern ts::auton skills;
    extern ts::auton sawp_dsr;
    extern ts::auton sawp_no_dsr;
}


#endif //AUTONS_H
