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
        pos starting_location(-48.7, -16.1, 0);
        pos match_loader_neg_neg_prime(-45, -41.5, 90);
        pos long_goal_neg_neg(-29.5, -49, 90);
        pos block_blip_neg_neg(-22.75, -15.5, 180);
        pos block_blip_neg_pos(-22.75, 21.5, 0);
        pos middle_goal_neg_pos(-8.5, 16.5, 135);
        pos primer_match_loader_neg_pos(-45, 52.5, 90);
        pos long_goal_neg_pos(-29, 52, 90);

        namespace dsr
        {
            pos push_point(-47, 4, 0);
            pos match_loader_neg_neg_prime(-45, -41.25, 90);
            pos long_goal_neg_neg(-29, -47.1, 90);
            pos block_blip_neg_neg(-22.75, -15, 180);
            pos block_blip_neg_pos(-22.0, 23.5, 0);
            pos middle_goal_neg_pos(-10.5, 16.5, 135);
            pos primer_match_loader_neg_pos(-45, 52.0, 90);
            pos long_goal_neg_pos(-29, 47.1, 90);
        }
    }
}

void sawp_auton()
{
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

    //There are 2 sections and each starts at 270 degrees.
    (void)dt->inertial.set_heading(0);
    dt->lem_chassis.setPose(POS(coords::sawp::starting_location));

    {
        conv->wings.toggle();
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(coords::sawp::match_loader_neg_neg_prime), 700, {.forwards = false, .minSpeed = 30}, false);
        chassis->swingToHeading(TPOS(coords::sawp::match_loader_neg_neg_prime), lemlib::DriveSide::LEFT, 700, {}, false);
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
        conv->match_loader.toggle();
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);//Not match loader just moving backwards
        pros::Task::delay(300);
        chassis->swingToHeading(195, lemlib::DriveSide::LEFT, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
        chassis->moveToPose(POS(coords::sawp::block_blip_neg_neg), 1000, {.forwards = false, .minSpeed = 30}, false);
        chassis->moveToPoint(MPOS(coords::sawp::block_blip_neg_pos), 1200, {.forwards = false, .maxSpeed = 75}, false);
        conv->match_loader.toggle();
    }

    {
        chassis->turnToHeading(90, 400, {}, false);
        chassis->moveToPose(POS(coords::sawp::middle_goal_neg_pos), 700, {}, false);
        (void)conv->exhaust.move(0.5 * FULL_POWER);
        pros::Task::delay(375);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPoint(MPOS(coords::sawp::primer_match_loader_neg_pos), 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 20}, false);
        chassis->swingToHeading(TPOS(coords::sawp::primer_match_loader_neg_pos), lemlib::DriveSide::RIGHT, 500, {}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(1000);
    }

    {
        chassis->moveToPoint(MPOS(coords::sawp::long_goal_neg_pos), 1000, {.minSpeed = 30}, false);
        chassis->tank(70, 70, true);
        (void)conv->exhaust.move(FULL_POWER);
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

    (void)dt->inertial.set_heading(0);
    dt->lem_chassis.setPose(0,0,0);

    dt->l_chassis.reset_location_force(NEG_NEG);

    if (push)
    {
        chassis->moveToPoint(MPOS(push_point), 550, {}, false);
    }

    {
        conv->wings.toggle();
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(match_loader_neg_neg_prime), 1900, {.forwards = false, .minSpeed = 30}, false);
        chassis->swingToHeading(TPOS(match_loader_neg_neg_prime), lemlib::DriveSide::LEFT, 700, {}, false);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(1000);
    }

    {
        dt->l_chassis.reset_location_force(NEG_NEG);
    }

    {
        chassis->moveToPoint(MPOS(long_goal_neg_neg), 900, {.forwards = true, .minSpeed = 30}, false);
        chassis->tank(70, 70, true);
        (void)conv->exhaust.move(FULL_POWER);
        pros::Task::delay(1250);
    }

    {
        conv->match_loader.toggle();
    }

    {
        dt->l_chassis.reset_location_force(NEG_NEG);
    }

    {
        chassis->swingToHeading(205, lemlib::DriveSide::LEFT, 700, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
        dt->l_chassis.reset_location_force(NEG_NEG);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        chassis->moveToPose(POS(block_blip_neg_neg), 1000, {.forwards = false, .lead = 0.2, .minSpeed = 30}, false);
        chassis->moveToPoint(MPOS(block_blip_neg_pos), 1800, {.forwards = false, .maxSpeed = 85}, false);
    }

    {
        chassis->turnToHeading(120, 400, {}, false);
        chassis->moveToPose(POS(middle_goal_neg_pos), 700, {.lead = 0.2}, false);
        (void)conv->exhaust.move(0.5 * FULL_POWER);
        pros::Task::delay(375);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(primer_match_loader_neg_pos), 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 20}, false);
        chassis->swingToHeading(TPOS(primer_match_loader_neg_pos), lemlib::DriveSide::RIGHT, 500, {}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(1000);
    }

    {
        dt->l_chassis.reset_location_force(NEG_POS);
    }

    {
        chassis->moveToPoint(MPOS(long_goal_neg_pos), 1000, {.minSpeed = 30}, false);
        chassis->tank(70, 70, true);
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

ts::auton autons::sawp_dsr = ts::auton("SAWP DSR", sawp_dsr_auton);
ts::auton autons::sawp_dsr_push = ts::auton("SAWP DSR PUSH", sawp_dsr_auton_push);
ts::auton autons::sawp_no_dsr = ts::auton("SAWP", sawp_auton);
