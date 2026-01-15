//
// Created by aaron on 1/14/2026.
//

#include "../../include/autons.hpp"
#include "../../include/titanselect/titanselect.hpp"
#include "../../include/subsystems/drivetrain.hpp"
#include "../../include/subsystems/conveyor.hpp"
#include "../../include/pros/adi.hpp"
#include "../../include/pros/misc.hpp"
#include "../../include/pros/rtos.hpp"
#include "../../include/pros/motors.hpp"

namespace coords
{
    namespace elims
    {
        namespace left
        {

        }

        namespace right
        {

        }
    }
}

void elims_left_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -55;
    constexpr auto LONG_GOAL = 40;
    constexpr auto EXHAUST_INDEX = -0.2 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    (void)dt->inertial.set_heading(270);
    dt->lem_chassis.setPose(-46.15,12.80,270);

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPoint(-26.5, 18.5, 600, {.forwards = false, .minSpeed = 30}, false);
        chassis->moveToPoint(-5, 42.5, 900, {.forwards = false}, false);
    }

    {
        chassis->moveToPose(-7, 8.5, 135, 1700, {.lead = 0.7}, true);
        {
            pros::Task::delay(1000);
            (void)conv->conveyor_intake.move(-0.5 * FULL_POWER);
            pros::Task::delay(300);
            (void)conv->conveyor_intake.brake();
            chassis->waitUntilDone();
        }
    }

    {
        (void)conv->conveyor_intake.move(0.65 * FULL_POWER);
        (void)conv->exhaust.move(-0.8 * FULL_POWER);
        conv->trapdoor.toggle();
        pros::Task::delay(2100);
        conv->trapdoor.toggle();
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        conv->match_loader.toggle();
    }

    {
        chassis->moveToPose(-57, 47.5, 90, 1500, {.forwards = false, .lead = 0.5,.minSpeed = 20}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(700);
    }

    {
        chassis->moveToPose(-25, 46.5, 90, 1500, {.minSpeed = 30}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1500);
        conv->match_loader.toggle();
    }

    {
        chassis->moveToPoint(-36, 55, 700, {.forwards = false, .minSpeed = 30}, false);
        chassis->moveToPose(-12, 58, 90, 1000, {}, false);
        chassis->moveToPose(-6, 58, 90, 1000, {}, false);
        chassis->tank(0, 0, true);
    }

}


void elims_right_auton()
{

}

ts::auton autons::elims_left = ts::auton("ELIM LEFT", elims_left_auton);
ts::auton autons::elims_right = ts::auton("ELIM RIGHT", elims_left_auton);