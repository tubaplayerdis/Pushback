//
// Created by aaron on 1/10/2026.
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
    namespace sawp
    {
        namespace dsr
        {
            pos start_push(-46.89,-7.5,0);
            pos start_normal(-47.13,-15.5,0);
            pos push_point(-47, 4, 0);
            pos match_loader_neg_neg_prime(-46.3, -45.25, 90);
            pos match_loader_neg_neg_prime_push(-46.3, -45.25, 90);
            pos long_goal_neg_neg(-29, -47.1, 90);
            pos block_blip_neg_pos(-23.5, 22.5, 0);
            pos middle_goal_neg_pos(-10.5, 10.5, 135);
            pos match_loader_neg_pos(-51, 41.5, 90);
            pos long_goal_neg_pos(-29, 47.1, 90);
        }

        namespace counter
        {
            pos push_point(-47, 4, 0);
            pos match_loader_neg_neg_prime(-46.3, -40.0, 90);
            pos match_loader_neg_neg_prime_push(-46.3, -40.0, 90);
            pos long_goal_neg_neg(-29, -47.1, 90);
            pos block_blip_neg_neg(-24.5, -15, 180);
            pos block_blip_neg_pos(-23.5, 19.0, 145);
            pos long_goal_prime(-40, 41.5, 90);
            pos middle_goal_neg_pos(-10.0, 9.00, 135);
            pos match_loader_neg_pos(-56, 47.1, 90);
            pos long_goal_neg_pos(-29, 47.1, 90);
        }
    }
}

void sawp_dsr_counter_auton_raw(bool push)
{
    using namespace coords::sawp::counter;

    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -40;
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

    dt->l_chassis.perform_dsr_init(NEG_NEG, 180);

    dt->l_chassis.start_location_recording("SAWP C ");

    conv->wings.toggle();

    if (push)
    {
        chassis->moveToPoint(MPOS(push_point), 550, {.forwards = false}, false);
    }

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        conv->match_loader.toggle();
        if (push)
        {
            chassis->moveToPoint(MPOS(match_loader_neg_neg_prime_push), 2500, {.maxSpeed = 80, .minSpeed = 80, .earlyExitRange = 1}, false);
        } else
        {
            chassis->moveToPoint(MPOS(match_loader_neg_neg_prime), 2500, {.maxSpeed = 80, .minSpeed = 80, .earlyExitRange = 1}, false);
        }
        chassis->turnToHeading(TPOS(match_loader_neg_neg_prime), 700, {}, false);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(1100);
    }

    {
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
    }

    {
        chassis->moveToPoint(MPOS(long_goal_neg_neg), 900, {.forwards = true, .earlyExitRange = 1}, false);
        chassis->tank(30, 30, true);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(750);
    }

    {
        conv->match_loader.toggle();
    }

    {
        chassis->turnToPoint(MPOS(block_blip_neg_pos), 700, {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 90, .earlyExitRange = 1}, false);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        chassis->moveToPose(POS(block_blip_neg_pos), 1300, {.forwards = false, .horizontalDrift = 8, .lead = 0.2, .minSpeed = 60, .earlyExitRange = 5}, false);
    }

    {
        chassis->moveToPoint(MPOS(long_goal_prime), 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 4}, false);
        chassis->turnToHeading(TPOS(long_goal_prime), 500, {}, false);
        chassis->moveToPoint(MPOS(long_goal_neg_pos), 700, {.minSpeed = 40, .earlyExitRange = 1}, false);
    }

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1250);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        dt->l_chassis.perform_dsr_quad(NEG_POS);
    }

    {
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(match_loader_neg_pos), 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(300);
    }

    {
        dt->l_chassis.perform_dsr_quad(NEG_POS);
    }

    {
        chassis->moveToPose(POS(middle_goal_neg_pos), 1500, {.lead = 0.1, .minSpeed = 60, .earlyExitRange = 5}, true);
        {
            pros::Task::delay(1000);
            (void)conv->match_loader.toggle();
            (void)conv->trapdoor.retract();
            conv->conveyor_intake.move(-FULL_POWER);
            pros::Task::delay(200);
            conv->conveyor_intake.brake();
            chassis->waitUntilDone();
        }

        {
            (void)conv->exhaust.move(-FULL_POWER * 0.65);
            (void)conv->conveyor_intake.move(FULL_POWER * 0.55);
        }

        {
            chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            (void)dt->lem_drivetrain.rightMotors->brake();
            (void)dt->lem_drivetrain.leftMotors->brake();
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
    constexpr auto EXHAUST_INDEX = -0.3 * FULL_POWER;
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

    if (push)
    {
        chassis->setPose(POS(start_push));
    }
    else
    {
        chassis->setPose(POS(start_push));
    }

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
            chassis->moveToPoint(MPOS(match_loader_neg_neg_prime_push), 2500, {.forwards = false}, false);
        } else
        {
            chassis->moveToPoint(MPOS(match_loader_neg_neg_prime), 2500, {.forwards = false}, false);
        }
        chassis->turnToHeading(TPOS(match_loader_neg_neg_prime), 700, {}, false);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(1200);
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
        chassis->moveToPoint(MPOS(block_blip_neg_pos), 2500, {.forwards = false}, true);
        {
            chassis->waitUntil(34);
            conv->match_loader.toggle();
            chassis->waitUntilDone();
        }
    }

    {
        chassis->turnToPoint(MPOS(middle_goal_neg_pos), 500, {.minSpeed = 40, .earlyExitRange = 0.1}, false);
        chassis->moveToPose(POS(middle_goal_neg_pos), 700, {.lead = 0.2}, true);
        {
            pros::Task::delay(400);
            (void)conv->trapdoor.retract();
            conv->conveyor_intake.move(-FULL_POWER);
            pros::Task::delay(120);
            conv->conveyor_intake.brake();
            chassis->waitUntilDone();
        }
        (void)conv->exhaust.move(-FULL_POWER * 0.65);
        (void)conv->conveyor_intake.move(FULL_POWER * 0.7);
        pros::Task::delay(1000);
        (void)conv->trapdoor.extend();
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPoint(MPOS(match_loader_neg_pos), 1000, {.forwards = false}, false);
        chassis->turnToHeading(90, 500, {}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(1100);
    }

    {
        dt->l_chassis.perform_dsr();
    }

    {
        chassis->moveToPoint(MPOS(long_goal_neg_pos), 1000, {.minSpeed = 60, .earlyExitRange = 1}, false);
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
ts::auton autons::sawp_dsr_counter_push = ts::auton("SAWP C PUSH", sawp_dsr_counter_auton_push);
