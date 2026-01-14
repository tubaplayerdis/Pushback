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
    constexpr auto MATCH_LOADER = -80;
    constexpr auto LONG_GOAL = 10;
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
    dt->lem_chassis.setPose(0,0,270);

    dt->l_chassis.reset_location_force(NEG_POS);

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        conv->wings.toggle();
    }

    {
        chassis->moveToPoint(-26.5, 18.5, 600, {.forwards = false, .minSpeed = 30}, false);
        chassis->moveToPoint(-8, 43.5, 800, {.forwards = false}, false);
    }

    {
        chassis->moveToPose(-8.5, 8.5, 135, 1500, {}, false);
    }

    {
        (void)conv->conveyor_intake.move(-0.8 * FULL_POWER);
        conv->trapdoor.toggle();
        pros::Task::delay(2000);
        conv->trapdoor.toggle();
        (void)conv->conveyor_intake.move(FULL_POWER);
        conv->match_loader.toggle();
    }

    {
        chassis->moveToPose(-56, 46.5, 90, 1500, {.forwards = false, .lead = 0.5,.minSpeed = 20}, false);
    }
}


void elims_right_auton()
{

}

ts::auton autons::elims_left = ts::auton("ELIM LEFT", elims_left_auton);
ts::auton autons::elims_right = ts::auton("ELIM RIGHT", elims_left_auton);