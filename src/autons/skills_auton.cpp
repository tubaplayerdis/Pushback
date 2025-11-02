//
// Created by aaron on 11/2/2025.
//
#include "../../include/autons.hpp"
#include "../../include/titanselect/titanselect.hpp"
#include "../../include/subsystems/drivetrain.hpp"
#include "../../include/subsystems/conveyor.hpp"
#include "../../include/pros/adi.hpp"
#include "../../include/pros/misc.hpp"
#include "../../include/pros/rtos.hpp"
#include "../../include/pros/motors.hpp"

void do_checkpoint_reset()
{

}

void quadrant_one_skills()
{

}

void skills_routine()
{
    do_checkpoint_reset();
    quadrant_one_skills();
}

ts::auton autons::skills = ts::auton("Skills", skills_routine);