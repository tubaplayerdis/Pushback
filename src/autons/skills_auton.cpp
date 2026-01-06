//
// Created by aaron on 11/2/2025.
//
#include "../../include/autons.hpp"
#include "../../include/titanselect/titanselect.hpp"
#include "../../include/subsystems/drivetrain.hpp"
#include "../../include/subsystems/conveyor.hpp"
#include "../../include/pros/adi.hpp"
#include "../../include/pros/misc.hpp"
#include "../../include/pros/rtos.hpp"
#include "../../include/pros/motors.hpp"

#define SECTION_1
#define SECTION_2
#define SECTION_3

namespace coords
{
    namespace segment_uno
    {
        pos red_block_blip_neg_pos(-32.63, 17.5, 220);
        pos middle_goal_pos(-8.5, 8.5, 135);
        pos match_loader_neg_pos(-54, 46, 90);
        pos neg_pos_trans_pose(-24, 56, 90);
        pos neg_pos_trans_point(33, 56.5, 90);
        pos long_goal_pos_pos(25, 47, 270);
        pos match_loader_pos_pos(59, 46.5, 270);
    }

    namespace segment_dos
    {
        pos parking_zone_blue(65, 10, 90);
        pos red_block_blip_pos_neg(30, -15, 35);
        pos middle_goal_neg(8.5, -8.5, 315);
    }

    namespace segment_tres
    {
        pos match_loader_pos_neg(56, -44.5, 270);
        pos pos_neg_trans_pose(24, -57.5, 270);
        pos pos_neg_trans_point(-35, -55.5, 270);
        pos long_goal_neg_neg(-25, -47, 90);
        pos match_loader_neg_neg(-59, -46.5, 90);
        pos parking_zone_red(-65, -10, 180);
    }
}

namespace power_values
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -55;
    constexpr auto LONG_GOAL = 30;
}

void skills_routine()
{
    using namespace coords::segment_uno;
    using namespace power_values;

    //Get drivetrain object
    localization* dt  = localization::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    localization* lc = localization::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    //There are 2 sections and each starts at 270 degrees.
    (void)dt->inertial.set_heading(270);
    dt->lem_chassis.setPose(0,0,270);

#ifdef SECTION_1

    {
        dt->l_chassis.reset_location_force(NEG_POS);
    }

    {
        (void)conv->wings.toggle();
        (void)conv->exhaust.brake();
        (void)conv->conveyor_intake.move(FULL_POWER);
    }

    {
        chassis->moveToPoint(-32.63, 17.5, 800, {.forwards = false, .maxSpeed = 50}, false);
        pros::Task::delay(200);
        (void)conv->conveyor_intake.brake();
    }

    {
        chassis->turnToHeading(90, 900, {.maxSpeed = 100}, false);
        (void)conv->conveyor_intake.move(FULL_POWER);
    }
        //-15.5, 14

    {
        chassis->moveToPose(-8.5, 8.5, 135, 1500, {}, false);
        (void)conv->exhaust.move(FULL_POWER * 0.4);
        pros::Task::delay(500);
        (void)conv->exhaust.brake();
    }

    {
        conv->match_loader.toggle();
        (void)conv->exhaust.move(-0.3 * FULL_POWER);
        chassis->moveToPose(-55.1, 45.5, 90, 2000, {.forwards = false, .lead = 0.35, .maxSpeed = 90}, false);
    }

    {
        dt->l_chassis.reset_location_normal(NEG_POS, NEG_POS);
    }

    {
        pros::Task::delay(2000);
        (void)conv->conveyor_intake.brake();
    }

    {
        chassis->moveToPose(-24, 57, 90, 1000, {.minSpeed = 40}, false);
        conv->match_loader.toggle();
        chassis->moveToPoint(35, 56.5, 1500, {.minSpeed = 40}, false);
        chassis->swingToHeading(270, lemlib::DriveSide::RIGHT, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    }

    {
        dt->l_chassis.reset_location_normal(POS_POS, POS_NEG);
    }

    {
        chassis->moveToPose(25, 47, 270, 900, {}, false);
    }

    {
        chassis->tank(10, 10, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(1500);
    }

    {
        conv->match_loader.toggle();
        (void)conv->exhaust.move(-0.2 * FULL_POWER);
        chassis->moveToPose(59.0, 46.5, 270, 1000, { .forwards = false }, false);
        chassis->tank(-40, -40, true);
        pros::Task::delay(200);
        chassis->tank(0,0,true);
        pros::Task::delay(1800);
    }

    {
        dt->l_chassis.reset_location_normal(POS_POS, POS_NEG);
    }

    {
        chassis->moveToPose(25, 47, 270, 1000, {}, false);
    }

    {
        chassis->tank(10, 10, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(1500);
        (void)conv->exhaust.move(-0.2 * FULL_POWER);
    }

    {
        chassis->tank(-40, -40, true);
        pros::Task::delay(500);
        chassis->tank(40, 40, true);
        pros::Task::delay(500);
        conv->match_loader.toggle();
    }

#endif

    /*
     *END SECTION 1
     *START SECTION 2 at the goal.
     */

#ifdef SECTION_2

    {
        (void)conv->exhaust.move(-0.3 * FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        dt->l_chassis.reset_location_normal(POS_POS, POS_NEG);
    }

    {
        chassis->moveToPose(67, 12, 0, 1400, {.forwards = false, .lead = 0.1, .minSpeed = 40, .earlyExitRange = 0.5}, false);
        chassis->tank(-90, -86, true);
        pros::Task::delay(1100);
        pros::Task::delay(300);
    }

    {
        chassis->swingToHeading(115, lemlib::DriveSide::LEFT, 2000, {}, false);
    }

    {
        dt->l_chassis.reset_location_force(POS_NEG);
        (void)conv->conveyor_intake.move(FULL_POWER);
    }

    {
        chassis->moveToPose(30, -15, 35, 2000, {.forwards = false}, false);
        chassis->turnToHeading(225, 1000, {}, false);
        chassis->moveToPose(8.5, -8.5, 315, 2000, {}, false);
    }

    {
        (void)conv->conveyor_intake.move(-FULL_POWER);
        pros::Task::delay(200);
    }

    {
        (void)conv->exhaust.move(-FULL_POWER * 0.85);
        (void)conv->conveyor_intake.move(FULL_POWER * 0.5);
        (void)conv->trapdoor.extend();
        pros::Task::delay(3500);
    }


    //section 2 still needs to have the goal scoring and positioning done

#endif

#ifdef SECTION_3

    //Starts at the middle goal at approximately 8.5, -8.5

#ifndef SECTION_2 //If section 2 is undefined and section 3 is defined, run the debug init
    chassis->setPose(8.5, -8.5, 315);
#endif



    {
        (void)conv->trapdoor.toggle(); //When debugging this segment only remove this line
        conv->wings.extend();
        conv->match_loader.extend();
        (void)conv->conveyor_intake.move(FULL_POWER);
        (void)conv->exhaust.move(FULL_POWER);
    }

    {
        chassis->moveToPose(POS(coords::segment_tres::match_loader_pos_neg), 1800, {.forwards = false, .lead = 0.35, .maxSpeed = 90}, false);
        (void)conv->exhaust.move(-FULL_POWER * 0.3);
        dt->l_chassis.reset_location_normal(POS_NEG, POS_NEG);
        pros::Task::delay(2000);
    }

    {
        chassis->moveToPose(POS(coords::segment_tres::pos_neg_trans_pose), 1000, {.minSpeed = 40}, false);
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(coords::segment_tres::pos_neg_trans_point), 1500, {.minSpeed = 40}, false);
        chassis->swingToHeading(85, lemlib::DriveSide::RIGHT, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    }

    {
        dt->l_chassis.reset_location_force(NEG_NEG);
    }

    {
        chassis->moveToPose(POS(coords::segment_tres::long_goal_neg_neg), 1000, {}, false);
        chassis->tank(10, 10, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(1500);
        dt->l_chassis.reset_location_force(NEG_NEG);
    }

    {
        conv->match_loader.toggle();
        (void)conv->exhaust.move(-FULL_POWER * 0.3);
        chassis->moveToPose(POS(coords::segment_tres::match_loader_neg_neg), 1500, {.forwards = false}, false);
        pros::Task::delay(2000);
        dt->l_chassis.reset_location_force(NEG_NEG);
    }

    {
        chassis->moveToPose(POS(coords::segment_tres::long_goal_neg_neg), 1500, {}, false);
        chassis->tank(10, 10, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(1500);
        dt->l_chassis.reset_location_force(NEG_NEG);
    }

    {
        chassis->tank(-40, -40, true);
        pros::Task::delay(500);
        chassis->tank(40, 40, true);
        pros::Task::delay(500);
        conv->match_loader.toggle();
    }

    {
        chassis->moveToPose(POS(coords::segment_tres::parking_zone_red), 1500, {.forwards = false, .lead = 0.1, .minSpeed = 40, .earlyExitRange = 0.5}, false);
        chassis->tank(-90, -90, true);
        pros::Task::delay(700);
        chassis->tank(0, 0, true);
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

ts::auton autons::skills = ts::auton("Skills", skills_routine);