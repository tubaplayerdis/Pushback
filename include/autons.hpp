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

void skills_routine();

namespace autons
{
    extern ts::auton skills;
    extern ts::auton sawp_dsr;
    extern ts::auton sawp_dsr_push;
    extern ts::auton sawp_dsr_counter;
    extern ts::auton sawp_dsr_counter_push;
    extern ts::auton elims_left_dsr;
    extern ts::auton elims_left_fast;
    extern ts::auton elims_left_fast_fast;
    extern ts::auton elims_left_middle_end;
    extern ts::auton elims_right_middle_fast;
    extern ts::auton elims_right_middle_slow;
    extern ts::auton elims_right;
    extern ts::auton elims_right_middle_low;
}


#endif //AUTONS_H
