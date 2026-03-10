//
// Created by aaron on 11/2/2025.
//
#include "../../include/autons.hpp"
#include "../../include/titanselect/titanselect.hpp"
#include "../../include/subsystems/localization.hpp"
#include "../../include/subsystems/conveyor.hpp"
#include "../../include/pros/adi.hpp"
#include "../../include/pros/misc.hpp"
#include "../../include/pros/rtos.hpp"
#include "../../include/pros/motors.hpp"
#include <fstream>

//#define SECTION_1
#define SECTION_2
//#define SECTION_3
//#define SECTION_4

namespace coords
{
    namespace segment_uno
    {
        pos red_block_blip_neg_pos(-26.00, 23.5, 220);
        pos middle_goal_pos(-9.5, 10.5, 135);
        pos match_loader_neg_pos(-55, 47, 90);
        pos neg_pos_trans_pose(-24, 62, 90);
        pos neg_pos_trans_point(33.5, 62, 90);
        pos long_goal_pos_pos(29.0, 47.1, 270);
        pos match_loader_pos_pos(58.0, 47.0, 270);
    }

    namespace segment_dos
    {
        pos parking_zone_blue(64.5, 16.5, 0);
        pos block_quad_pos_pos(18, 19.5, 155);
        pos red_block_blip_pos_neg(30.75, -17.75, 35);
        pos middle_goal_neg(4.5, -4.5, 315);
        pos middle_goal_neg_inner(1.5, -1.5, 315);
        pos match_loader_pos_neg(48, -41.0, 270);
    }

    namespace segment_tres
    {
        pos pos_neg_trans_pose(24, -61.5, 270);
        pos pos_neg_trans_point(-35, -63, 270);
        pos long_goal_neg_neg(-25, -46.75, 90);
        pos match_loader_neg_neg(-58, -46.75, 90);
        pos long_goal_neg_neg_two(-25, -46.75, 90);
    }

    namespace segment_quad
    {
        pos parking_zone_red(-69, -13.25, 180);
    }
}

namespace power_values
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -40;
    constexpr auto LONG_GOAL = 30;
    constexpr auto EXHAUST_INDEX = -0.25 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;
    constexpr auto SCORING_TIME = 1800;
    constexpr auto MATCHLOADING_TIME = 1800;
}

void skills_routine()
{
    using namespace coords::segment_uno;
    using namespace power_values;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    //There are 2 sections and each starts at 270 degrees.

    chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    dt->l_chassis.start_location_recording("SKILLS");

#ifdef SECTION_1

    {
        dt->l_chassis.perform_dsr_init(NEG_NEG, 180);
    }

    {
        (void)conv->wings.toggle();
        (void)conv->exhaust.move(EXHAUST_INDEX);
        (void)conv->conveyor_intake.move(FULL_POWER);
    }

    {
        chassis->tank(-55, -52, true);
        pros::Task::delay(1400);
        conv->match_loader.toggle();
        pros::Task::delay(550);
    }

    {
        chassis->swingToHeading(90, lemlib::DriveSide::LEFT, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 100}, true);
        {
            pros::Task::delay(1000);
            conv->match_loader.toggle();
        }
        dt->l_chassis.perform_dsr_quad(NEG_POS);
    }

    {
        chassis->moveToPoint(-10.5, 10.5, 2000, {.forwards = true, .maxSpeed = 70}, false);
        pros::Task::delay(200);
    }

    {
        chassis->turnToPoint(0, 0, 800, {.minSpeed = 100, .earlyExitRange = 1}, false);
        chassis->moveToPoint(-18.5, 18.5, 1500, {.forwards = false, .maxSpeed = 60}, false);
    }
        //-15.5, 14

    {
        chassis->moveToPoint(MPOS(coords::segment_uno::middle_goal_pos), 1500, {}, false);
    }

    {
        (void)conv->conveyor_intake.move(-FULL_POWER);
        (void)conv->exhaust.move(-FULL_POWER);
        pros::Task::delay(150);
    }

    {
        (void)conv->exhaust.move(-FULL_POWER * 0.8);
        (void)conv->conveyor_intake.move(FULL_POWER * 0.65);
        (void)conv->trapdoor.extend();
        pros::Task::delay(2700);
    }

    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        conv->trapdoor.retract();
    }

    {
        conv->exhaust.move(FULL_POWER);
        conv->match_loader.toggle();
        chassis->moveToPose(POS(match_loader_neg_pos), 2000, {.forwards = false, .horizontalDrift = 2, .lead = 0.25}, false);
    }

    {
        conv->exhaust.move(EXHAUST_INDEX);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(MATCHLOADING_TIME);
    }

    {
        dt->l_chassis.perform_dsr_quad(NEG_POS);
    }

    {
        chassis->moveToPose(POS(coords::segment_uno::neg_pos_trans_pose), 1000, {.minSpeed = 40}, false);
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(coords::segment_uno::neg_pos_trans_point), 2000, {}, false);

        {
           //dt->l_chassis.perform_dsr_quad(POS_POS);
        }

        chassis->swingToHeading(270, lemlib::DriveSide::RIGHT, 1500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 70}, false);
    }

    {
        dt->l_chassis.perform_dsr_quad(POS_POS);
    }

    {
        chassis->moveToPoint(MPOS(coords::segment_uno::long_goal_pos_pos), 800, {.minSpeed = 50, .earlyExitRange = 0.5}, false);
    }

    {
        chassis->tank( LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        //chassis->turnToHeading(270, 1000, {.minSpeed = 127, .earlyExitRange = 10}, true);
        pros::Task::delay(SCORING_TIME);
    }

    {
        conv->match_loader.toggle();
        (void)conv->exhaust.move(EXHAUST_INDEX);
        chassis->moveToPoint(MPOS(coords::segment_uno::match_loader_pos_pos), 1500, { .forwards = false, .maxSpeed = 70, .earlyExitRange = 4 }, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(MATCHLOADING_TIME);
    }

    {
        dt->l_chassis.perform_dsr_quad(POS_POS);
    }

    {
        chassis->moveToPoint(MPOS(coords::segment_uno::long_goal_pos_pos), 1500, {.minSpeed = 50}, false);
    }

    {
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(SCORING_TIME);
    }

    {
        conv->match_loader.toggle();
    }

#endif

    /*
     *END SECTION 1
     *START SECTION 2 at the goal.
     */

#ifdef SECTION_2

#ifndef SECTION_1
    // SECTION 2 STARTS AT AROUND 80 PSI

    dt->l_chassis.perform_dsr_init(POS_POS, 270);
#endif


    {
        (void)conv->conveyor_intake.move(FULL_POWER);
        //dt->l_chassis.perform_dsr_quad(POS_POS);
        conv->wings.extend();
    }

    {
        chassis->moveToPose(POS(coords::segment_dos::parking_zone_blue), 2000, {.forwards = false, .horizontalDrift = 10, .lead = 0.5, .maxSpeed = 70, .minSpeed = 30, .earlyExitRange = 1}, false);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        chassis->tank(-55, -52, true);
        pros::Task::delay(1400);
        conv->match_loader.toggle();
        pros::Task::delay(600);
    }

    {
        chassis->swingToHeading(90, lemlib::DriveSide::LEFT, 1500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
        conv->match_loader.toggle();
    }

    {
        dt->l_chassis.perform_dsr_quad(POS_NEG);
        (void)conv->conveyor_intake.move(FULL_POWER);
    }

    {
        chassis->moveToPoint(MPOS(coords::segment_dos::block_quad_pos_pos), 3000, {.forwards = false, .maxSpeed = 90}, false);
        chassis->turnToHeading(46, 1000, {}, false);
        chassis->moveToPoint(16, 16, 1000, {.forwards = false}, false);
        conv->descore.toggle();
        chassis->moveToPoint(11, 11, 1000, {.forwards = false}, false);
    }

    {
        (void)conv->conveyor_intake.move(-FULL_POWER);
        pros::Task::delay(500);
        conv->do_low_goal_macro = true;
        pros::Task::delay(3500);
        conv->do_low_goal_macro = false;
        conv->descore.toggle();
    }

    {
        chassis->moveToPoint(15, 20, 800, {}, false);
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
        chassis->turnToPoint(MPOS(coords::segment_dos::match_loader_pos_neg), 1000, {.forwards = false}, false);
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(coords::segment_dos::match_loader_pos_neg), 2000, {.forwards = false, .maxSpeed = 100}, false);
        chassis->swingToHeading(270, lemlib::DriveSide::RIGHT, 1000, {}, false);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        pros::Task::delay(MATCHLOADING_TIME);
    }

    //section 2 still needs to have the goal scoring and positioning done

#endif

#ifdef SECTION_3

    //Starts at the middle goal at approximately 8.5, -8.5

#ifndef SECTION_2 //If section 2 is undefined and section 3 is defined, run the debug init

#endif

    {
        dt->l_chassis.perform_dsr_quad(POS_NEG);
        chassis->moveToPose(POS(coords::segment_tres::pos_neg_trans_pose), 1000, {.minSpeed = 40}, false);
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(coords::segment_tres::pos_neg_trans_point), 1700, {.minSpeed = 40}, false);

        {
            dt->l_chassis.perform_dsr_quad(NEG_NEG);
        }

        chassis->swingToPoint(MPOS(coords::segment_tres::long_goal_neg_neg), lemlib::DriveSide::RIGHT, 1700, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}, false);
    }

    {
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
    }

    {
        chassis->moveToPoint(MPOS(coords::segment_tres::long_goal_neg_neg), 800, {.minSpeed = 50, .earlyExitRange = 0.5}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        //chassis->turnToHeading(90, 1000, {.minSpeed = 127, .earlyExitRange = 10}, true);
        pros::Task::delay(SCORING_TIME);
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
    }

    {
        conv->match_loader.toggle();
        (void)conv->exhaust.move(-FULL_POWER * 0.2);
        chassis->moveToPoint(MPOS(coords::segment_tres::match_loader_neg_neg), 1500, {.forwards = false, .maxSpeed = 70, .earlyExitRange = 4}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(MATCHLOADING_TIME);
    }

    {
        chassis->moveToPose(POS(coords::segment_tres::long_goal_neg_neg_two), 1500, {.lead = 0.1, .minSpeed = 50, .earlyExitRange = 5}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(SCORING_TIME);
    }

#endif

#ifdef SECTION_4

#ifndef SECTION_3
    (void)dt->inertial.set_heading(90);
    dt->lem_chassis.setPose(0,0,90);
    dt->l_chassis.reset_location_force(NEG_NEG);
#endif

    {
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        (void)conv->conveyor_intake.move(FULL_POWER);
        conv->match_loader.retract();
    }

    {
        chassis->moveToPose(POS(coords::segment_quad::parking_zone_red), 1600, {.forwards = false, .lead = 0.35, .minSpeed = 30}, false);
        chassis->tank(-50, -50, true);
        pros::Task::delay(900);
        chassis->tank(NO_POWER, NO_POWER, true);
    }

#endif
    //29.61, 42.62

    while (true)
    {
        lemlib::Pose pose = chassis->getPose();
        //controller_master.print(1,0, "%s", sup.c_str());
        controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
        pros::Task::delay(50);
    }
}

ts::auton autons::skills = ts::auton("SKILLZ", skills_routine);