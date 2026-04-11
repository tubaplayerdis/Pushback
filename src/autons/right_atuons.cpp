//
// Created by aaron on 4/11/2026.
//

#include "../../include/autons.hpp"
#include "../../include/titanselect/titanselect.hpp"
#include "../../include/subsystems/localization.hpp"
#include "../../include/subsystems/conveyor.hpp"
#include "../../include/pros/adi.hpp"
#include "../../include/pros/misc.hpp"
#include "../../include/pros/motors.h"
#include "../../include/pros/rtos.hpp"
#include "../../include/pros/motors.hpp"

namespace coords
{
    namespace elims
    {
        namespace right
        {
            pos block_blip_trio(-29.0, -18.0, 0);
            pos long_goal_uno(-25, -47.0, 90);
            pos match_loader(-52, -43.5, 85);
            pos wing_forward_final(-18, -36, 90);
        }

        namespace right_fast_fast
        {
            pos block_blip_trio(-29.0, -19.0, 0);
            pos long_goal_uno(-25, -47.0, 90);
            pos match_loader(-46, -43.5, 90);
            pos wing_forward_final(-17, -35.5, 90);
        }
    }
}

void elims_right_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -50;
    constexpr auto LONG_GOAL = 30;
    constexpr auto EXHAUST_INDEX = -0.2 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;

    using namespace coords::elims::right;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    dt->l_chassis.perform_dsr_init(NEG_NEG, 270);

    //dt->l_chassis.start_location_recording("ELIMS RIGHT DSR ");

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        conv->wings.toggle();
    }

    {
        chassis->moveToPoint(MPOS(block_blip_trio), 800, {.forwards = false, .minSpeed = 127, .earlyExitRange = 3}, false);
        conv->match_loader.toggle();
        chassis->swingToPoint(MPOS(match_loader), lemlib::DriveSide::LEFT, 1000, {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 100, .earlyExitRange = 5}, false);
    }

    {
        chassis->moveToPose(POS(match_loader), 2000, {.forwards = false, .horizontalDrift = 6, .lead = 0.25, .minSpeed = 40}, false);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        pros::Task::delay(700);
    }

    {
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
    }

    {
        chassis->moveToPoint(MPOS(long_goal_uno), 900, {.minSpeed = 100, .earlyExitRange = 4}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1250);
        conv->match_loader.toggle();
    }

    {
        conv->wings.toggle();
        conv->exhaust.move(EXHAUST_INDEX);
        chassis->turnToHeading(150, 700, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 90}, false);
        chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 700, {.maxSpeed = 90}, false);
        chassis->moveToPoint(MPOS(wing_forward_final), 1500, {.minSpeed = 40}, false);
        chassis->tank(0, 0, true);
        chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
}

void elims_right_fast_fast_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -50;
    constexpr auto LONG_GOAL = 127;
    constexpr auto EXHAUST_INDEX = -0.2 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;

    using namespace coords::elims::right_fast_fast;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    dt->l_chassis.perform_dsr_init(NEG_NEG, 270);

    dt->l_chassis.start_location_recording("ELIMS RIGHT DSR ");

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        conv->wings.toggle();
    }

    {
        chassis->moveToPoint(MPOS(block_blip_trio), 1000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 3}, false);
        conv->match_loader.toggle();
        chassis->swingToPoint(MPOS(match_loader), lemlib::DriveSide::LEFT, 1000, {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 127, .earlyExitRange = 5}, false);
    }

    {
        conv->match_loader.toggle();
        chassis->turnToHeading(240,800, {.minSpeed = 80}, false);
        chassis->swingToHeading(90, lemlib::DriveSide::LEFT, 900, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 110}, false);
    }

    {
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 750, {}, false);
        pros::Task::delay(750);
    }

    {
        conv->wings.toggle();
        conv->exhaust.move(EXHAUST_INDEX);
        chassis->turnToHeading(150, 600, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
        chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 500, {}, false);
        chassis->moveToPoint(MPOS(wing_forward_final), 1500, {.minSpeed = 40}, false);
        chassis->tank(0, 0, true);
        chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
}

ts::auton autons::elims_right_fast_fast = ts::auton("R_4_4S", elims_right_fast_fast_auton);
ts::auton autons::elims_right = ts::auton("R_7_6S", elims_right_auton);