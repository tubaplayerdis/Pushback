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

#include "../../include/pros/motors.h"

//#define SECTION_1
#define SECTION_2
#define SECTION_3

namespace coords
{
    namespace segment_uno
    {
        pos red_block_blip_neg_pos(-26.00, 23.5, 220);
        pos middle_goal_pos(-10.0, 10.0, 135);
        pos match_loader_neg_pos(-56, 47.25, 90);
        pos neg_pos_trans_pose(-24, 62, 90);
        pos neg_pos_trans_point(29.5, 61, 90);
        pos long_goal_neg_pos_prime(-35.0, 47.1, 90);
        pos long_goal_neg_pos(-29.0, 47.1, 90);
        pos long_goal_pos_pos(29.0, 47.1, 270);
        pos match_loader_pos_pos(56.0, 47.1, 270);
    }

    namespace segment_dos
    {
        pos parking_zone_blue(64.0, 14.0, 0);
        pos block_quad_pos_pos(20.5, 19.0, 155);
        pos red_block_blip_pos_neg(30.75, -17.75, 35);
        pos block_quad(23.5, -23.5, 325);
        pos long_goal_pos_neg_prime(37.0, -46.0, 90);
        pos long_goal_pos_neg(29.0, -47.1, 90);
        pos match_loader_pos_neg(56, -47.1, 270);
    }

    namespace segment_tres
    {
        pos pos_neg_trans_pose(24, -62, 270);
        pos pos_neg_trans_point(-29.5, -61, 270);
        pos long_goal_neg_neg(-25, -47.1, 90);
        pos match_loader_neg_neg(-56, -47.1, 90);
        pos parking_zone_red(-63.0, -14.5, 180);
    }
}

namespace power_values
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -40;
    constexpr auto LONG_GOAL = 30;
    constexpr auto EXHAUST_INDEX = -0.20 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.35 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;
    constexpr auto SCORING_TIME = 1350;
    constexpr auto MATCHLOADING_TIME = 1400;
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
        dt->l_chassis.perform_dsr_init(NEG_POS, 0);
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
        pros::Task::delay(500);
    }

    {
        chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
        (void)dt->lem_drivetrain.rightMotors->brake();
        (void)dt->lem_drivetrain.leftMotors->brake();
        pros::Task::delay(200);
        chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
        chassis->turnToPoint(-21.5, 6.75, 1500, {}, false);
        conv->match_loader.toggle();
        (void)conv->conveyor_intake.brake();
    }

    {
        chassis->moveToPoint(-21.5, 6.75, 1500, {.maxSpeed = 90}, false);
    }

    {
        conv->conveyor_intake.move(FULL_POWER);
        chassis->swingToHeading(135, lemlib::DriveSide::RIGHT, 800, {}, false);
        chassis->moveToPoint(-15.5, 15.5, 1500, {.forwards = false, .maxSpeed = 60}, false);
    }
        //-15.5, 14

    {
        chassis->moveToPoint(MPOS(coords::segment_uno::middle_goal_pos), 1500, {}, false);
        chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        (void)dt->lem_drivetrain.rightMotors->brake();
        (void)dt->lem_drivetrain.leftMotors->brake();
    }

    {
        pros::Task::delay(200);
        (void)conv->trapdoor.retract();//seats a block in
        (void)conv->low_goal.extend();//activate to keep blocks in ideal position.
        (void)conv->conveyor_intake.move(-FULL_POWER);
        (void)conv->exhaust.move(-FULL_POWER * 0.20);
        pros::Task::delay(115);
    }

    {
        (void)conv->exhaust.move(-FULL_POWER * 0.65);
        (void)conv->conveyor_intake.move_velocity(600.0 * 0.40);
        pros::Task::delay(1500);
        (void)conv->exhaust.move(-FULL_POWER * 0.35);
        (void)conv->conveyor_intake.move_velocity(600.0 * 0.30);
        pros::Task::delay(3500);
        chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    }

    {
        conv->trapdoor.extend();
        conv->low_goal.retract();//deactive
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(EXHAUST_INDEX);
    }

    {
        chassis->moveToPoint(MPOS(long_goal_neg_pos_prime), 1500, {.forwards = false}, true);
        {
            pros::Task::delay(500);
            conv->match_loader.toggle();
            chassis->waitUntilDone();
        }
        chassis->turnToPoint(MPOS(long_goal_neg_pos), 750, {}, false);
        chassis->moveToPoint(MPOS(long_goal_neg_pos), 750, {.minSpeed = 40}, false);
        {
            chassis->tank(LONG_GOAL, LONG_GOAL, true);
            (void)conv->exhaust.move(FULL_POWER);
            (void)conv->conveyor_intake.move(FULL_POWER);
            pros::Task::delay(650);
            (void)conv->exhaust.move(EXHAUST_INDEX);
            (void)conv->conveyor_intake.brake();

            dt->l_chassis.perform_dsr();
        }
        chassis->moveToPoint(MPOS(match_loader_neg_pos), 1000, {.forwards = false, .earlyExitRange = 2}, false);
        (void)conv->conveyor_intake.move(FULL_POWER);
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
        chassis->moveToPoint(MPOS(coords::segment_uno::neg_pos_trans_point), 1500, {}, false);

        {
           //dt->l_chassis.perform_dsr_quad(POS_POS);
        }

        chassis->swingToHeading(230, lemlib::DriveSide::RIGHT, 750, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 100, .earlyExitRange = 1},false);
        chassis->turnToHeading(270, 750, {}, false);

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
        chassis->moveToPoint(MPOS(coords::segment_uno::match_loader_pos_pos), 1000, { .forwards = false, .earlyExitRange = 2}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(MATCHLOADING_TIME);
    }

    {
        dt->l_chassis.perform_dsr_quad(POS_POS);
    }

    {
        chassis->moveToPoint(MPOS(coords::segment_uno::long_goal_pos_pos), 1000, {.minSpeed = 60, .earlyExitRange = 1}, false);
    }

    {
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(SCORING_TIME);
    }

    {
        conv->match_loader.toggle();
        dt->l_chassis.perform_dsr();
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
        chassis->moveToPoint(MPOS(coords::segment_dos::block_quad_pos_pos), 3000, {.forwards = false, .maxSpeed = 85}, false);
        chassis->turnToHeading(44.0, 1000, {}, false);
        chassis->moveToPoint(15, 15, 1000, {.forwards = false}, false);
        (void)conv->conveyor_intake.move(NO_POWER);
        conv->descore.toggle();
        conv->low_goal.toggle();
        chassis->moveToPoint(10.5, 10.5, 750, {.forwards = false}, false);
    }

    {
        (void)conv->conveyor_intake.move(-FULL_POWER);
        pros::Task::delay(900);
        conv->do_low_goal_macro = true;
        pros::Task::delay(3100);
        conv->do_low_goal_macro = false;
        conv->descore.toggle();
        conv->low_goal.toggle();
    }

    {
        chassis->moveToPoint(15, 20, 500, {}, false);
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
        chassis->turnToPoint(MPOS(coords::segment_dos::block_quad), 500, {.forwards = false}, false);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        chassis->moveToPoint(MPOS(coords::segment_dos::block_quad), 1000, {.forwards = false, .earlyLambdaRange = 8, .earlyLambda = [conv]() -> void {conv->match_loader.toggle();}}, false);
        chassis->turnToPoint(MPOS(coords::segment_dos::long_goal_pos_neg_prime), 500, {.forwards = false}, false);
        chassis->moveToPoint(MPOS(coords::segment_dos::long_goal_pos_neg_prime), 1000, {.forwards = false}, false);
        chassis->turnToPoint(MPOS(coords::segment_dos::long_goal_pos_neg), 500, {}, false);
        chassis->moveToPoint(MPOS(coords::segment_dos::long_goal_pos_neg), 500, {}, false);
        //chassis->swingToHeading(270, lemlib::DriveSide::RIGHT, 1000, {}, false);
    }

    {
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(650);
    }

    {
        (void)conv->exhaust.move(EXHAUST_INDEX);
        //dt->l_chassis.perform_dsr();
        chassis->moveToPoint(MPOS(coords::segment_dos::match_loader_pos_neg), 1000, {.forwards = false, .earlyExitRange = 4}, false);
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
    dt->l_chassis.perform_dsr_init(POS_NEG, 270);
    conv->match_loader.toggle();
#endif

    {
        dt->l_chassis.perform_dsr_quad(POS_NEG);
        chassis->moveToPose(POS(coords::segment_tres::pos_neg_trans_pose), 1000, {.minSpeed = 40}, false);
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(coords::segment_tres::pos_neg_trans_point), 1500, {}, false);

        chassis->swingToHeading(50, lemlib::DriveSide::RIGHT, 750, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 100, .earlyExitRange = 1},false);
        chassis->turnToHeading(90, 750, {}, false);
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
        //chassis->swingToHeading(90, lemlib::DriveSide::LEFT, 1000, {}, true);
        pros::Task::delay(SCORING_TIME);
        dt->l_chassis.perform_dsr_quad(NEG_NEG);
    }

    {
        conv->match_loader.toggle();
        (void)conv->exhaust.move(EXHAUST_INDEX);
        chassis->moveToPoint(MPOS(coords::segment_tres::match_loader_neg_neg), 1000, {.forwards = false, .earlyExitRange = 2}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(MATCHLOADING_TIME);
    }

    {
        chassis->moveToPoint(MPOS(coords::segment_tres::long_goal_neg_neg), 1000, {.minSpeed = 50}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(SCORING_TIME);
    }

    {
        //dt->l_chassis.perform_dsr_quad(NEG_NEG);
        (void)conv->exhaust.move(EXHAUST_INDEX);
        (void)conv->conveyor_intake.move(FULL_POWER);
        conv->match_loader.retract();
    }

    {
        chassis->moveToPose(POS(coords::segment_tres::parking_zone_red), 1600, {.forwards = false, .lead = 0.35, .minSpeed = 60}, false);
        chassis->tank(-90, -90, true);
        pros::Task::delay(500);
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