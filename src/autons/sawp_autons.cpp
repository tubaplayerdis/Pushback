//
// Created by aaron on 1/10/2026.
//

#include "../../include/autons.hpp"
#include "../../include/titanselect/titanselect.hpp"
#include "../../include/subsystems/drivetrain.hpp"
#include "../../include/subsystems/localization.hpp"
#include "../../include/subsystems/conveyor.hpp"
#include "../../include/pros/adi.hpp"
#include "../../include/pros/misc.hpp"
#include "../../include/pros/rtos.hpp"
#include "../../include/pros/motors.hpp"

namespace coords
{
    namespace sawp
    {
        namespace dsr
        {
            pos push_point(-47, 4, 0);
            pos match_loader_neg_neg_prime(-47.3, -38.50, 90);
            pos match_loader_neg_neg_prime_push(-47.3, -40.25, 90);
            pos long_goal_neg_neg(-29, -47.1, 90);
            pos block_blip_neg_neg(-22.75, -15, 180);
            pos block_blip_neg_pos(-24, 24.5, 0);
            pos middle_goal_neg_pos(-8.0, 10.5, 135);
            pos match_loader_neg_pos(-55, 47.6, 90);
            pos long_goal_neg_pos(-29, 47.6, 90);
        }

        namespace counter
        {
            pos push_point(-47, 4, 0);
            pos match_loader_neg_neg_prime(-47.3, -39.00, 90);
            pos match_loader_neg_neg_prime_push(-47.3, -40.25, 90);
            pos long_goal_neg_neg(-29, -47.1, 90);
            pos block_blip_neg_neg(-22.75, -15, 180);
            pos block_blip_neg_pos(-24, 37.5, 0);
            pos middle_goal_neg_pos(-8.0, 8.0, 135);
            pos match_loader_neg_pos(-55, 47.6, 90);
            pos long_goal_neg_pos(-29, 47.6, 90);
        }
    }
}

void sawp_dsr_counter_auton_raw(bool push)
{
    using namespace coords::sawp::counter;

    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -45;
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

    dt->l_chassis.perform_dsr_init(NEG_NEG, 0);

    dt->l_chassis.start_location_recording("SAWP C ");

    if (push)
    {
        chassis->moveToPoint(MPOS(push_point), 550, {}, false);
    }

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        conv->match_loader.toggle();
        if (push)
        {
            chassis->moveToPoint(MPOS(match_loader_neg_neg_prime_push), 2500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 1}, false);
        } else
        {
            chassis->moveToPoint(MPOS(match_loader_neg_neg_prime), 2500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 1}, false);
        }
        chassis->swingToHeading(TPOS(match_loader_neg_neg_prime), lemlib::DriveSide::LEFT, 700, {}, false);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(1000);
    }

    {
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
    }

    {
        chassis->moveToPoint(MPOS(long_goal_neg_neg), 900, {.forwards = true, .minSpeed = 60, .earlyExitRange = 1}, false);
        chassis->tank(70, 70, true);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1250);
    }

    {
        conv->match_loader.toggle();
    }

    {
        chassis->turnToPoint(MPOS(block_blip_neg_pos), 1000, {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE, .earlyExitRange = 1}, false);
        {
            //dt->l_chassis.perform_dsr_quad(NEG_NEG);
        }
        (void)conv->exhaust.move(EXHAUST_INDEX);
        chassis->moveToPose(POS(block_blip_neg_neg), 1000, {.forwards = false, .lead = 0.2, .minSpeed = 50}, false);
        chassis->moveToPoint(MPOS(block_blip_neg_pos), 2500, {.forwards = false, .maxSpeed = 85}, true);
        {
            chassis->waitUntil(38);
            conv->match_loader.toggle();
            pros::Task::delay(100);
            conv->match_loader.toggle();
            chassis->waitUntilDone();
        }
    }

    {
        chassis->turnToHeading(280,1000, {.minSpeed = 80}, false);
        conv->match_loader.toggle();
        chassis->swingToHeading(90, lemlib::DriveSide::RIGHT, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 30, .earlyExitRange = 1}, false);
    }

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        chassis->turnToHeading(90, 1000, {.minSpeed = 100, .earlyExitRange = 5}, true);
        pros::Task::delay(1500);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        dt->l_chassis.perform_dsr_quad(NEG_POS);
    }

    {
        chassis->moveToPoint(MPOS(match_loader_neg_pos), 1500, {.forwards = false, .minSpeed = 60}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(900);
    }

    {
        dt->l_chassis.perform_dsr_quad(NEG_POS);
    }

    {
        chassis->moveToPoint(MPOS(middle_goal_neg_pos), 1500, {.minSpeed = 60, .earlyExitRange = 6}, true);
        {
            pros::Task::delay(500);
            conv->exhaust.brake();
            conv->conveyor_intake.move(-FULL_POWER);
            pros::Task::delay(120);
            conv->conveyor_intake.brake();
            chassis->waitUntilDone();
        }

        {
            (void)conv->exhaust.move(-FULL_POWER * 0.8);
            (void)conv->conveyor_intake.move(FULL_POWER * 0.65);
            (void)conv->trapdoor.extend();
        }
    }

}

void sawp_dsr_auton_raw(bool push)
{
    using namespace coords::sawp::dsr;

    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -45;
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

    dt->l_chassis.perform_dsr_init(NEG_NEG, 0);

    dt->l_chassis.start_location_recording("SAWP");

    if (push)
    {
        chassis->moveToPoint(MPOS(push_point), 550, {}, false);
    }

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        conv->match_loader.toggle();
        if (push)
        {
            chassis->moveToPoint(MPOS(match_loader_neg_neg_prime_push), 2500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 1}, false);
        } else
        {
            chassis->moveToPoint(MPOS(match_loader_neg_neg_prime), 2500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 1}, false);
        }
        chassis->swingToHeading(TPOS(match_loader_neg_neg_prime), lemlib::DriveSide::LEFT, 700, {}, false);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(1000);
    }

    {
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
    }

    {
        chassis->moveToPoint(MPOS(long_goal_neg_neg), 900, {.forwards = true, .minSpeed = 60, .earlyExitRange = 1}, false);
        chassis->tank(70, 70, true);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1250);
    }

    {
        conv->match_loader.toggle();
    }

    {
        chassis->turnToPoint(MPOS(block_blip_neg_pos), 1000, {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE, .earlyExitRange = 1}, false);
        {
            //dt->l_chassis.perform_dsr_quad(NEG_NEG);
        }
        (void)conv->exhaust.move(EXHAUST_INDEX);
        chassis->moveToPose(POS(block_blip_neg_neg), 1000, {.forwards = false, .lead = 0.2, .minSpeed = 50}, false);
        chassis->moveToPoint(MPOS(block_blip_neg_pos), 1800, {.forwards = false, .maxSpeed = 85}, true);
        {
            chassis->waitUntil(42);
            conv->match_loader.toggle();
            chassis->waitUntilDone();
        }
    }

    {
        chassis->turnToPoint(MPOS(middle_goal_neg_pos), 500, {.minSpeed = 40, .earlyExitRange = 0.1}, false);
        chassis->moveToPose(POS(middle_goal_neg_pos), 700, {.lead = 0.2}, true);
        {
            conv->exhaust.brake();
            conv->conveyor_intake.move(-FULL_POWER);
            pros::Task::delay(120);
            conv->conveyor_intake.brake();
            chassis->waitUntilDone();
        }
        (void)conv->exhaust.move(-FULL_POWER * 0.8);
        (void)conv->conveyor_intake.move(FULL_POWER * 0.65);
        (void)conv->trapdoor.extend();
        pros::Task::delay(900);
        (void)conv->trapdoor.retract();
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPose(POS(match_loader_neg_pos), 1500, {.forwards = false, .horizontalDrift = 2, .lead = 0.25, .minSpeed = 60}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(1000);
    }

    {
        dt->l_chassis.perform_dsr_quad(NEG_POS);
    }

    {
        chassis->moveToPoint(MPOS(long_goal_neg_pos), 1000, {.minSpeed = 60, .earlyExitRange = 4}, false);
        chassis->tank(30, 30, true);
        (void)conv->exhaust.move(FULL_POWER);
    }
}

void sawp_dsr_auton()
{
    sawp_dsr_auton_raw(false);
}

void sawp_dsr_auton_push()
{
    sawp_dsr_auton_raw(true);
}

void sawp_dsr_counter_auton()
{
    sawp_dsr_counter_auton_raw(false);
}

void sawp_dsr_counter_auton_push()
{
    sawp_dsr_counter_auton_raw(true);
}

ts::auton autons::sawp_dsr = ts::auton("SAWP", sawp_dsr_auton);
ts::auton autons::sawp_dsr_push = ts::auton("SAWP P", sawp_dsr_auton_push);
ts::auton autons::sawp_dsr_counter = ts::auton("SAWP C", sawp_dsr_counter_auton);
ts::auton autons::sawp_dsr_counter_push = ts::auton("SAWP CP", sawp_dsr_counter_auton);
