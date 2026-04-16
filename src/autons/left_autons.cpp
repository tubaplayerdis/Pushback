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
        namespace left
        {
            namespace dsr
            {
                pos block_blip_trio(-26.5, 18.5, 0);
                pos block_blip_duo(-6, 42.5, 0);
                pos middle_goal_high(-10.0, 10.0, 130);
                pos match_loader(-49, 42.50, 90);
                pos long_goal(-25, 47.1, 90);
                pos wing_prime_back(-36, 55, 0);
                pos wing_forward_final(-10, 57.7, 90);
            }
        }

        namespace left_fast
        {
            pos block_blip_trio(-29.0, 19.0, 0);
            pos match_loader(-56, 44.0, 90);
            pos long_goal(-25, 47.25, 90);
            pos wing_forward_final(-10, 57.5, 90);
        }

        namespace left_fast_fast
        {
            pos block_blip_trio(-29.0, 19.0, 0);
            pos long_goal_uno(-25, 47.0, 90);
            pos match_loader(-46, 43.5, 90);
            pos wing_forward_final(-10, 58.5, 90);
        }

        namespace left_middle_end
        {
            pos match_loader(-47, 47, 90);
            pos long_goal(-29, 47.1, 90);
            pos block_blip_trio(-20.0, 20.0, 0);
            pos middle_goal(-10.5, 10.5, 135);
            pos wing_end(-10, 61, 90);
        }
    }
}

void elims_left_middle_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -55;
    constexpr auto LONG_GOAL = 20;
    constexpr auto EXHAUST_INDEX = -0.2 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;

    using namespace coords::elims::left_middle_end;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    dt->l_chassis.perform_dsr_init(NEG_POS, 0);

    dt->l_chassis.start_location_recording("middle");

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        conv->match_loader.toggle();
    }

    {
        chassis->moveToPoint(MPOS(match_loader), 2000, {}, false);
        chassis->turnToHeading(TPOS(match_loader), 500, {}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(850);
    }

    {
        dt->l_chassis.perform_dsr();
    }

    {
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(long_goal), 1000, {.minSpeed = 60, .earlyExitRange = 1 }, false);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(750);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        dt->l_chassis.perform_dsr();
    }

    {
        chassis->turnToPoint(MPOS(block_blip_trio), 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 1}, false);
        chassis->moveToPoint(MPOS(block_blip_trio), 750, {.forwards = false}, false);
        chassis->turnToPoint(MPOS(middle_goal), 750, {}, false);
        chassis->moveToPoint(MPOS(middle_goal), 1000, {.earlyExitRange = 3}, true);
        {
            conv->conveyor_intake.move(-FULL_POWER);
            pros::Task::delay(150);
            conv->conveyor_intake.brake();
            (void)conv->trapdoor.extend();
            chassis->waitUntilDone();
        }
    }

    {
        (void)conv->exhaust.move(-FULL_POWER * 0.65);
        (void)conv->conveyor_intake.move(FULL_POWER * 0.7);
        pros::Task::delay(2000);
        (void)conv->trapdoor.retract();
    }

    {
        chassis->moveToPoint(-43, 61.0, 2000, {.forwards = false}, false);
        chassis->turnToHeading(90, 500, {}, false);
        chassis->moveToPoint(MPOS(wing_end), 2000, {}, false);
    }
}

void elims_left_dsr_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -55;
    constexpr auto LONG_GOAL = 20;
    constexpr auto EXHAUST_INDEX = -0.2 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;

    using namespace coords::elims::left::dsr;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    dt->l_chassis.perform_dsr_init(NEG_POS, 270);
    //dt->lem_chassis.setPose(-45.85, 12.09, 270);

    dt->l_chassis.start_location_recording("Whatever");

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPoint(MPOS(block_blip_trio), 600, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2.5}, false);
        conv->match_loader.toggle();
        chassis->swingToPoint(-5, -5, lemlib::DriveSide::RIGHT, 700, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 30}, false);
        //chassis->moveToPoint(MPOS(block_blip_duo), 900, {.forwards = false}, false);
    }

    {
        chassis->moveToPose(POS(middle_goal_high), 800, {.horizontalDrift = 8, .lead = 0.1, .earlyExitRange = 1}, true);
        {
            pros::Task::delay(500);
            (void)conv->conveyor_intake.move(-FULL_POWER);
            pros::Task::delay(200);
            (void)conv->conveyor_intake.brake();
            chassis->waitUntilDone();
        }
    }

    {
        conv->trapdoor.toggle();
        (void)conv->exhaust.move(-0.8 * FULL_POWER);
        (void)conv->conveyor_intake.move(0.65 * FULL_POWER);
        pros::Task::delay(1000);
        conv->trapdoor.toggle();
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPoint(MPOS(match_loader), 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 4}, false);
        chassis->turnToHeading(TPOS(match_loader), 500, {}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(800);
    }

    {
        dt->l_chassis.perform_dsr();
    }

    {
        chassis->moveToPoint(MPOS(long_goal), 1000, {.minSpeed = 50, .earlyExitRange = 4}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1000);
        conv->match_loader.toggle();
    }

    {
        conv->exhaust.move(EXHAUST_INDEX);
        chassis->turnToHeading(150, 400, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
        chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 400, {}, false);
        chassis->moveToPoint(MPOS(wing_forward_final), 1500, {.minSpeed = 40}, false);
        chassis->tank(0, 0, true);
        chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
}

void elims_left_fast_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -50;
    constexpr auto LONG_GOAL = 30;
    constexpr auto EXHAUST_INDEX = -0.2 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;

    using namespace coords::elims::left_fast;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    dt->l_chassis.perform_dsr_init(NEG_POS, 270);

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        conv->wings.toggle();
    }

    {
        chassis->moveToPoint(MPOS(block_blip_trio), 800, {.forwards = false, .minSpeed = 127, .earlyExitRange = 3}, false);
        conv->match_loader.toggle();
        chassis->swingToPoint(MPOS(match_loader), lemlib::DriveSide::RIGHT, 1000, {.forwards = false, .direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 127, .earlyExitRange = 5}, false);
    }

    {
        chassis->moveToPose(POS(match_loader), 1200, {.forwards = false, .horizontalDrift = 1, .lead = 0.25, .minSpeed = 90}, false);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        pros::Task::delay(800);
    }

    {
        dt->l_chassis.perform_dsr();
    }

    {
        chassis->moveToPoint(MPOS(long_goal), 900, {.minSpeed = 100, .earlyExitRange = 4}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1250);
        conv->match_loader.toggle();
    }

    {
        conv->wings.toggle();
        conv->exhaust.move(EXHAUST_INDEX);
        chassis->turnToHeading(150, 400, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
        chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 400, {}, false);
        chassis->moveToPoint(MPOS(wing_forward_final), 1500, {.minSpeed = 40}, false);
        chassis->tank(0, 0, true);
        chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
}

void elims_left_fast_fast_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -50;
    constexpr auto LONG_GOAL = 30;
    constexpr auto EXHAUST_INDEX = -0.2 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;

    using namespace coords::elims::left_fast_fast;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    dt->lem_chassis.setPose(-45.85, 12.09, 270);

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        conv->wings.toggle();
    }

    {
        chassis->moveToPoint(MPOS(block_blip_trio), 800, {.forwards = false, .minSpeed = 127, .earlyExitRange = 3}, false);
        conv->match_loader.toggle();
        chassis->swingToPoint(MPOS(match_loader), lemlib::DriveSide::RIGHT, 1000, {.forwards = false, .direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 127, .earlyExitRange = 5}, false);
    }

    {
        conv->match_loader.toggle();
        chassis->turnToHeading(240,900, {.minSpeed = 80}, false);
        chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 900, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 110}, false);
    }

    {
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
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

ts::auton autons::elims_left_dsr = ts::auton("L_4+3_10S", elims_left_dsr_auton);
ts::auton autons::elims_left_fast = ts::auton("L_7_6S", elims_left_fast_auton);
ts::auton autons::elims_left_fast_fast = ts::auton("L_4_4S", elims_left_fast_fast_auton);
ts::auton autons::elims_left_middle_end = ts::auton("L_3+4_MID_12S", elims_left_middle_auton);