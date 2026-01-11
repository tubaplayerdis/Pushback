//
// Created by aaron on 1/10/2026.
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
    namespace sawp
    {
        pos starting_location(-48.5, -14.5, 180);
        pos match_loader_neg_neg(-57, -47.5, 90);
        pos block_blip_neg_neg(-25.5, -20.5, 125);
        pos long_goal_neg_neg(-29, -46.5, 90);
        pos block_blip_neg_pos(-24, 17, 0);
    }
}

void sawp_auton(bool b_dsr)
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -50;
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

    //There are 2 sections and each starts at 270 degrees.
    (void)dt->inertial.set_heading(270);
    dt->lem_chassis.setPose(0,0,270);

    if (b_dsr)
    {
        dt->l_chassis.reset_location_force(NEG_NEG);
    }
    else
    {
        dt->lem_chassis.setPose(POS(coords::sawp::starting_location));
    }

    {
        conv->wings.toggle();
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPoint(MPOS(coords::sawp::block_blip_neg_neg), 600, {.forwards = false, .earlyExitRange = 0.1}, false);
        conv->match_loader.toggle();
        chassis->swingToHeading(45, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
        chassis->moveToPose(POS(coords::sawp::match_loader_neg_neg), 1000, {.forwards = false}, false);
    }

    {
        dt->l_chassis.reset_location_force(NEG_NEG);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(1000);
    }

    {
        chassis->moveToPoint(MPOS(coords::sawp::long_goal_neg_neg), 900, {.forwards = true, .minSpeed = 30}, false);
        chassis->tank(70, 70, true);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1250);
    }

    {
        (void)conv->exhaust.move(EXHAUST_INDEX);
        chassis->swingToHeading(350, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(coords::sawp::block_blip_neg_pos), 2000, {.forwards = false}, false);
    }
}

void do_sawp_dsr()
{
    sawp_auton(true);
}

void do_sawp_no_dsr()
{
    sawp_auton(false);
}

ts::auton autons::sawp_dsr = ts::auton("SAWP DSR", do_sawp_dsr);
ts::auton autons::sawp_no_dsr = ts::auton("SAWP NO DSR", do_sawp_no_dsr);