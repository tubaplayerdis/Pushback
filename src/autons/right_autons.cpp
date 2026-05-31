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
#include "../../include/lemlib/timer.hpp"
#include "../../include/pros/abstract_motor.hpp"

namespace coords
{
    namespace elims
    {
        namespace right
        {
            pos block_blip_trio(-29.0, -19.0, 0);
            pos long_goal_uno(-25, -47.25, 90);
            pos match_loader(-49, -41.75, 90);
            pos wing_forward_final(-10, -36, 90);
        }

        namespace right_middle_slow_fast
        {
            pos match_loader_prime(-46.3, -47.5, 90);
            pos long_goal(-27, -47.25, 90);
            pos block_blip_trio(-17.0, -17.0, 230);
            pos wing_forward_final(-9, -37, 90);
            pos low_goal(-9.5, -9.25, 225);
        }

        namespace right_middle_low
        {
            pos match_loader(-47.3, -47.5, 90);
            pos block_trio_nn(-22.5, -22.5, 225);
            pos low_goal(-10, -10, 225);
            pos block_trio_np(-22.5, 22.5, 180);
            pos middle_goal(-10, 10, 135);
            pos middle_goal_prime(-17, 17, 135);
        }
    }
}

void quick_approach_right(lemlib::Chassis* chassis)
{
    pos match_loader_fast_right(chassis->getPose().x, -37.00, 90);
    conveyor* conv = conveyor::get();
    chassis->moveToPoint(MPOS(match_loader_fast_right), 1500, {.minSpeed = 80, .earlyExitRange = 4}, false);
    conv->match_loader.toggle();
    chassis->turnToHeading(TPOS(match_loader_fast_right), 500, {}, false);
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

    dt->l_chassis.start_location_recording("RIGHT");

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        conv->wings.toggle();
    }

    {
        /**
         * This genuine bullshit works because it wants to and sometimes it dosent want to
         */
        chassis->moveToPoint(MPOS(block_blip_trio), 800, {.forwards = false, .minSpeed = 127, .earlyExitRange = 3}, false);
        conv->match_loader.toggle();
        chassis->swingToHeading(80, lemlib::DriveSide::LEFT, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 127, .earlyExitRange = 1}, true);
        {
            chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            chassis->waitUntilDone();
        }
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
        (void)conv->exhaust.move(EXHAUST_INDEX);
        wing_movement(chassis, wing_forward_final);
    }
}

void elims_right_middle_fast_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -40;
    constexpr auto LONG_GOAL = 30;
    constexpr auto EXHAUST_INDEX = -0.2 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;

    using namespace coords::elims::right_middle_slow_fast;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    lemlib::Timer move_timer(11000);

    dt->l_chassis.perform_dsr_init(NEG_NEG, 180);

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        quick_approach_right(chassis);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(800);
    }

    {
        dt->l_chassis.perform_dsr();
    }

    {
        chassis->moveToPoint(MPOS(long_goal), 900, {.minSpeed = 60, .earlyExitRange = 1}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(800);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        conv->match_loader.toggle();
    }

    {
        wing_movement(chassis, wing_forward_final);
    }

    {
        while (!move_timer.isDone())
        {
            pros::Task::delay(50);
        }
    }

    {
        conv->wings.toggle();
        chassis->moveToPoint(-17.00, -37, 750, {.forwards = false, .minSpeed = 50, .earlyExitRange = 1}, false);
        chassis->swingToPoint(MPOS(block_blip_trio), lemlib::DriveSide::LEFT, 750, {.forwards = false, .minSpeed = 100, .earlyExitRange = 1}, false);
        chassis->moveToPoint(MPOS(low_goal), 1000, {.forwards = false, .maxSpeed = 70}, false);
        conv->low_goal.toggle();
        (void)conv->conveyor_intake.move(-FULL_POWER);
    }
}

void elims_right_middle_slow_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -40;
    constexpr auto LONG_GOAL = 30;
    constexpr auto EXHAUST_INDEX = -0.2 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;

    using namespace coords::elims::right_middle_slow_fast;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    dt->l_chassis.perform_dsr_init(NEG_NEG, 180);

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        quick_approach_right(chassis);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(800);
    }

    {
        dt->l_chassis.perform_dsr();
    }

    {
        chassis->moveToPoint(MPOS(long_goal), 900, {.forwards = true, .minSpeed = 60, .earlyExitRange = 1}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(800);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        conv->match_loader.toggle();
        //dt->l_chassis.perform_dsr();
    }

    {
        chassis->swingToPoint(MPOS(low_goal), lemlib::DriveSide::LEFT, 2000, {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80, .earlyExitRange = 5}, false);
        chassis->moveToPoint(MPOS(low_goal), 1500, {.forwards = false, .maxSpeed = 70}, false);
        conv->low_goal.toggle();
        (void)conv->conveyor_intake.move(-FULL_POWER);
        pros::Task::delay(1000);
    }

    {
        chassis->moveToPoint(-23, -23, 1000, {.minSpeed = 40, .earlyExitRange = 4}, false);
        chassis->swingToHeading(90, lemlib::DriveSide::LEFT, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
        chassis->moveToPoint(MPOS(wing_forward_final), 1500, {}, false);

    }

    {
        brake_chassis(chassis);
    }
}

void elims_right_middle_low_auton()
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -50;
    constexpr auto LONG_GOAL = 127;
    constexpr auto EXHAUST_INDEX = -0.15 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;

    using namespace coords::elims::right_middle_low;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    dt->l_chassis.perform_dsr_init(NEG_NEG, 180);

    {
        (void)conv->exhaust.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPoint(MPOS(match_loader), 2000, {}, false);
        conv->match_loader.toggle();
        chassis->turnToHeading(TPOS(match_loader), 500, {}, false);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(850);
    }

    {
        chassis->moveToPoint(MPOS(match_loader), 1000, {}, false);
        conv->match_loader.toggle();
        chassis->turnToPoint(MPOS(block_trio_nn), 1000, {.forwards = false}, false);
    }

    {
        chassis->moveToPoint(MPOS(block_trio_nn), 1000, {.forwards = false, .minSpeed = 40}, false);
        chassis->moveToPose(POS(low_goal), 1000, {.forwards = false}, false);
        conv->low_goal.toggle();
    }

    {
        (void)conv->conveyor_intake.move(-FULL_POWER);
        pros::Task::delay(1000);
        (void)conv->conveyor_intake.move(FULL_POWER);
    }

    {
        chassis->moveToPoint(MPOS(block_trio_nn), 500, {}, false);
        conv->low_goal.toggle();
        chassis->turnToPoint(MPOS(block_trio_np), 500, {.forwards = false}, false);
        chassis->moveToPoint(MPOS(block_trio_np), 1000, {.forwards = false}, false);
    }

    {
        chassis->turnToPoint(MPOS(middle_goal), 1000, {}, false);
        chassis->moveToPoint(MPOS(middle_goal), 1200, {}, true);
        {
            pros::Task::delay(500);
            conv->conveyor_intake.move(-FULL_POWER);
            pros::Task::delay(150);
            conv->conveyor_intake.brake();
            chassis->waitUntilDone();
        }
    }

    {
        (void)conv->trapdoor.retract();
        (void)conv->exhaust.move(-FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER * 0.7);
        pros::Task::delay(1000);
        (void)conv->trapdoor.extend();
        (void)conv->exhaust.brake();
        (void)conv->conveyor_intake.brake();
    }

    {
        chassis->moveToPoint(MPOS(middle_goal_prime), 1000, {.forwards = false}, false);
        conv->descore.toggle();
        chassis->moveToPoint(MPOS(middle_goal), 1200, {}, false);
    }

}

ts::auton autons::elims_right_middle_fast = ts::auton("R_4U_3L_F", elims_right_middle_fast_auton);
ts::auton autons::elims_right_middle_slow = ts::auton("R_4U_3L", elims_right_middle_slow_auton);
ts::auton autons::elims_right = ts::auton("R_7U", elims_right_auton);
ts::auton autons::elims_right_middle_low = ts::auton("R_3L_4M", elims_right_middle_low_auton);