//
// Created by aaron on 1/14/2026.
//

#include "../../include/autons.hpp"
#include "../../include/titanselect/titanselect.hpp"
#include "../../include/subsystems/drivetrain.hpp"
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
            pos starting_location(-46.15,12.80,270);
            pos block_blip_trio(-26.5, 18.5, 0);
            pos block_blip_duo(-5, 42.5, 0);
            pos middle_goal_high(-7, 8.5, 135);
            pos match_loader(-57, 47.5, 90);
            pos long_goal(-25, 46.5, 90);
            pos wing_prime_back(-36, 55, 0);
            pos wing_forward_init(-12, 58, 90);
            pos wing_forward_final(-6, 58, 90);

            namespace dsr
            {
                pos block_blip_trio(-26.5, 18.5, 0);
                pos block_blip_duo(-6, 42.5, 0);
                pos middle_goal_high(-6.5, 8.5, 135);
                pos match_loader(-57, 46.5, 90);
                pos long_goal(-25, 46.5, 90);
                pos wing_prime_back(-36, 55, 0);
                pos wing_forward_init(-12, 57.5, 90);
                pos wing_forward_final(-10, 57.5, 90);
            }
        }

        namespace right
        {
            pos block_blip_trio(-29.0, -19.0, 0);
            pos long_goal_uno(-25, -46.5, 90);
            pos match_loader(-61, -44.0, 90);
            pos wing_forward_final(-10, -37.5, 90);
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

    using namespace coords::elims::left;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    (void)dt->inertial.set_heading(270);
    dt->lem_chassis.setPose(POS(starting_location));

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPoint(MPOS(block_blip_trio), 600, {.forwards = false, .minSpeed = 30}, false);
        chassis->moveToPoint(MPOS(block_blip_duo), 900, {.forwards = false}, false);
    }

    {
        chassis->moveToPose(POS(middle_goal_high), 1700, {.lead = 0.7}, true);
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
        chassis->moveToPose(POS(match_loader), 1500, {.forwards = false, .lead = 0.5,.minSpeed = 20}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(800);
    }

    {
        chassis->moveToPose(POS(long_goal), 1500, {.minSpeed = 30}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1500);
        conv->match_loader.toggle();
    }

    {
        chassis->moveToPoint(MPOS(wing_prime_back), 700, {.forwards = false, .minSpeed = 30}, false);
        chassis->moveToPose(POS(wing_forward_init), 1000, {}, false);
        chassis->moveToPose(POS(wing_forward_final), 1000, {}, false);
        chassis->tank(0, 0, true);
    }
}

void elims_left_dsr_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -55;
    constexpr auto LONG_GOAL = 40;
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

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPoint(MPOS(block_blip_trio), 600, {.forwards = false, .minSpeed = 30, .earlyExitRange = 0.5}, false);
        conv->match_loader.toggle();
        chassis->swingToHeading(125, lemlib::DriveSide::RIGHT, 700, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 30}, false);
        //chassis->moveToPoint(MPOS(block_blip_duo), 900, {.forwards = false}, false);
    }

    {
        chassis->moveToPose(POS(middle_goal_high), 800, {.lead = 0.1, .earlyExitRange = 1}, true);
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
        chassis->moveToPose(POS(match_loader), 1500, {.forwards = false, .horizontalDrift = 2, .lead = 0.4, .minSpeed = 20}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(800);
        dt->l_chassis.perform_dsr_quad(NEG_POS);
    }

    {
        chassis->moveToPose(POS(long_goal), 1500, {.minSpeed = 30}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1500);
        dt->l_chassis.perform_dsr_quad(NEG_POS);
        conv->match_loader.toggle();
    }

    {
        chassis->swingToHeading(130, lemlib::DriveSide::LEFT, 500, {.minSpeed = 120, .earlyExitRange = 1}, false);
        chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 500, {.minSpeed = 120, .earlyExitRange = 1}, false);
        chassis->moveToPose(POS(wing_forward_final), 1000, {}, false);
        chassis->tank(0, 0, true);
    }
}

void elims_right_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -20;
    constexpr auto LONG_GOAL = 40;
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

    dt->l_chassis.start_location_recording("2/14/2026", "1012");

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        conv->wings.toggle();
    }

    {
        chassis->moveToPoint(MPOS(block_blip_trio), 600, {.forwards = false, .minSpeed = 80, .earlyExitRange = 0.5}, false);
        conv->match_loader.toggle();
        chassis->swingToHeading(80, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 80, .earlyExitRange = 5}, false);
    }

    {
        chassis->moveToPose(POS(match_loader), 1500, {.forwards = false, .lead = 0.3, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 0.5}, false);
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
        chassis->moveToPoint(MPOS(long_goal_uno), 1000, {.minSpeed = 30, .earlyExitRange = 0.5}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1500);
        conv->match_loader.toggle();
    }

    {
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
    }

    {
        conv->wings.toggle();
        chassis->swingToHeading(130, lemlib::DriveSide::LEFT, 500, {.minSpeed = 120, .earlyExitRange = 1}, false);
        chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 500, {.minSpeed = 120, .earlyExitRange = 1}, false);
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
        chassis->moveToPose(POS(wing_forward_final), 1000, {.minSpeed = 60, .earlyExitRange = 0.1}, false);
        chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
}

ts::auton autons::elims_left = ts::auton("ELIM LEFT", elims_left_auton);
ts::auton autons::elims_left_dsr = ts::auton("ELIM LEFT DSR", elims_left_dsr_auton);
ts::auton autons::elims_right = ts::auton("ELIM RIGHT DSR", elims_right_auton);