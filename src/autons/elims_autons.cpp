//
// Created by aaron on 1/14/2026.
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
                pos middle_goal_high(-8.5, 8.5, 135);
                pos match_loader(-51, 43.15, 90);
                pos long_goal(-25, 47.1, 90);
                pos wing_prime_back(-36, 55, 0);
                pos wing_forward_init(-12, 57.5, 90);
                pos wing_forward_final(-8, 61, 90);
            }
        }

        namespace right
        {
            pos block_blip_trio(-29.0, -19.0, 0);
            pos long_goal_uno(-25, -46.0, 90);
            pos match_loader(-52, -42.5, 90);
            pos wing_forward_final(-6, -35.5, 90);
        }
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

    dt->l_chassis.start_location_recording("ELIMS LEFT DSR");

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
        pros::Task::delay(1500);
        conv->trapdoor.toggle();
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPoint(MPOS(match_loader), 1000, {.forwards = false}, false);
        chassis->turnToHeading(TPOS(match_loader), 500, {}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(800);
        dt->l_chassis.perform_dsr_quad(NEG_POS);
    }

    {
        chassis->moveToPoint(MPOS(long_goal), 1000, {.minSpeed = 50, .earlyExitRange = 4}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1000);
        //dt->l_chassis.perform_dsr_quad(NEG_POS);
        conv->match_loader.toggle();
    }

    {
        conv->exhaust.move(EXHAUST_INDEX);
        chassis->swingToHeading(130, lemlib::DriveSide::LEFT, 500, {.minSpeed = 120, .earlyExitRange = 1}, false);
        chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 500, {.minSpeed = 120, .earlyExitRange = 1}, false);
        chassis->moveToPose(POS(wing_forward_final), 1500, {.minSpeed = 40}, false);
        chassis->tank(0, 0, true);
        chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
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

    dt->l_chassis.start_location_recording("ELIMS RIGHT DSR ");

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        conv->wings.toggle();
    }

    {
        chassis->moveToPoint(MPOS(block_blip_trio), 800, {.forwards = false, .minSpeed = 127, .earlyExitRange = 3}, false);
        conv->match_loader.toggle();
        chassis->swingToPoint(MPOS(match_loader), lemlib::DriveSide::LEFT, 1000, {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 127, .earlyExitRange = 5}, false);
    }

    {
        chassis->moveToPose(POS(match_loader), 1500, {.forwards = false, .horizontalDrift = 6, .lead = 0.25, .minSpeed = 80}, false);
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
        chassis->moveToPoint(MPOS(long_goal_uno), 1000, {.minSpeed = 127, .earlyExitRange = 4}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1250);
        conv->match_loader.toggle();
    }

    {
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
    }

    {
        conv->wings.toggle();
        conv->exhaust.move(EXHAUST_INDEX);
        chassis->swingToHeading(130, lemlib::DriveSide::LEFT, 500, {.minSpeed = 120, .earlyExitRange = 1}, false);
        chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 500, {.minSpeed = 120, .earlyExitRange = 1}, false);
        chassis->moveToPose(POS(wing_forward_final), 1500, {.minSpeed = 40}, false);
        chassis->tank(0, 0, true);
        chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
}

ts::auton autons::elims_left_dsr = ts::auton("GORDON", elims_left_dsr_auton);
ts::auton autons::elims_right = ts::auton("GERBERT", elims_right_auton);