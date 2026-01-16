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
#include <fstream>

#define SECTION_1
#define SECTION_2
#define SECTION_3

namespace coords
{
    namespace segment_uno
    {
        pos red_block_blip_neg_pos(-32.00, 17.5, 220);
        pos middle_goal_pos(-8.5, 8.5, 135);
        pos match_loader_neg_pos(-57, 46.5, 90);
        pos neg_pos_trans_pose(-24, 57, 90);
        pos neg_pos_trans_point(35, 57, 90);
        pos long_goal_pos_pos(25, 47, 270);
        pos match_loader_pos_pos(59.0, 46.5, 270);
    }

    namespace segment_dos
    {
        pos parking_zone_blue(65.5, 12, 0);
        pos red_block_blip_pos_neg(30.5, -15.5, 35);
        pos middle_goal_neg(8.5, -8.5, 315);
    }

    namespace segment_tres
    {
        pos match_loader_pos_neg(57, -46.5, 270);
        pos pos_neg_trans_pose(24, -58, 270);
        pos pos_neg_trans_point(-36, -56.75, 270);
        pos long_goal_neg_neg(-25, -46.8, 90);
        pos match_loader_neg_neg(-59, -46.5, 90);
        pos parking_zone_red(-67, -11, 180);
    }
}

namespace power_values
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 0;
    constexpr auto MATCH_LOADER = -80;
    constexpr auto LONG_GOAL = 10;
    constexpr auto EXHAUST_INDEX = -0.2 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_LOW = -0.75 * FULL_POWER;
    constexpr auto EXHAUST_SCORE_HIGH = FULL_POWER;
}

void brake_motors(lemlib::Chassis* chassis)
{
    (void)chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    (void)chassis->drivetrain.leftMotors->brake();
    (void)chassis->drivetrain.rightMotors->brake();
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

    chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    bool stop_loop = false;

    pros::Task debug_task([chassis, &stop_loop]() -> void
    {
        std::ofstream output("path_debug_output.txt");
        while (!stop_loop)
        {
            lemlib::Pose pose = chassis->getPose();
            output << pose.x << ", " << pose.y << ", " << pose.theta << "\n";
            pros::Task::delay(50);
        }
        output.close();
    });

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
        chassis->moveToPoint(MPOS(coords::segment_uno::red_block_blip_neg_pos), 800, {.forwards = false, .maxSpeed = 50}, false);
        pros::Task::delay(200);
        (void)conv->conveyor_intake.brake();
    }

    {
        chassis->turnToHeading(90, 900, {.maxSpeed = 100}, false);
        (void)conv->conveyor_intake.move(FULL_POWER);
    }
        //-15.5, 14

    {
        chassis->moveToPose(POS(coords::segment_uno::middle_goal_pos), 1500, {}, false);
        (void)conv->exhaust.move(FULL_POWER * 0.4);
        pros::Task::delay(500);
        (void)conv->exhaust.brake();
    }

    {
        conv->match_loader.toggle();
        (void)conv->exhaust.move(-0.2 * FULL_POWER);
        chassis->moveToPose(POS(coords::segment_uno::match_loader_neg_pos), 2000, {.forwards = false, .lead = 0.50, .maxSpeed = 90}, false);
    }

    {
        dt->l_chassis.reset_location_normal(NEG_POS, NEG_POS);
    }

    {
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(200);
        brake_motors(chassis);
        pros::Task::delay(1800);
    }

    {
        chassis->moveToPose(POS(coords::segment_uno::neg_pos_trans_pose), 1000, {.minSpeed = 40}, false);
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(coords::segment_uno::neg_pos_trans_point), 1700, {.minSpeed = 40}, false);
        chassis->swingToHeading(270, lemlib::DriveSide::RIGHT, 900, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    }

    {
        dt->l_chassis.reset_location_normal(POS_POS, POS_NEG);
    }

    {
        chassis->moveToPose(POS(coords::segment_uno::long_goal_pos_pos), 900, {}, false);
    }

    {
        chassis->tank( LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(2000);
    }

    {
        conv->match_loader.toggle();
        (void)conv->exhaust.move(-0.2 * FULL_POWER);
        chassis->moveToPose(POS(coords::segment_uno::match_loader_pos_pos), 1000, { .forwards = false }, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(200);
        brake_motors(chassis);
        pros::Task::delay(1800);
    }

    {
        dt->l_chassis.reset_location_normal(POS_POS, POS_NEG);
    }

    {
        chassis->moveToPose(POS(coords::segment_uno::long_goal_pos_pos), 1000, {}, false);
    }

    {
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(2000);
        (void)conv->exhaust.move(-0.2 * FULL_POWER);
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

    {
        (void)conv->exhaust.move(-0.2 * FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        dt->l_chassis.reset_location_normal(POS_POS, POS_NEG);
    }

    {
        chassis->moveToPose(POS(coords::segment_dos::parking_zone_blue), 1400, {.forwards = false, .lead = 0.1, .minSpeed = 75, .earlyExitRange = 0.5}, false);
        chassis->tank(-91, -86, true);
        pros::Task::delay(800);
        conv->match_loader.toggle();
        pros::Task::delay(300);
        conv->match_loader.toggle();
    }

    {
        chassis->swingToHeading(115, lemlib::DriveSide::LEFT, 2000, {}, false);
    }

    {
        dt->l_chassis.reset_location_force(POS_NEG);
        (void)conv->conveyor_intake.move(FULL_POWER);
    }

    {
        chassis->moveToPose(POS(coords::segment_dos::red_block_blip_pos_neg), 2000, {.forwards = false}, false);
        chassis->turnToHeading(225, 1000, {}, false);
        chassis->moveToPose(POS(coords::segment_dos::middle_goal_neg), 2000, {}, false);
    }

    {
        (void)conv->conveyor_intake.move(-FULL_POWER);
        pros::Task::delay(300);
    }

    {
        (void)conv->exhaust.move(-FULL_POWER * 0.75);
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
        chassis->moveToPose(POS(coords::segment_tres::match_loader_pos_neg), 2000, {.forwards = false, .lead = 0.50, .maxSpeed = 90}, false);
        (void)conv->exhaust.move(-FULL_POWER * 0.2);
        dt->l_chassis.reset_location_normal(POS_NEG, POS_NEG);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(200);
        brake_motors(chassis);
        pros::Task::delay(1800);
    }

    {
        chassis->moveToPose(POS(coords::segment_tres::pos_neg_trans_pose), 1000, {.minSpeed = 40}, false);
        conv->match_loader.toggle();
        chassis->moveToPoint(MPOS(coords::segment_tres::pos_neg_trans_point), 1700, {.minSpeed = 40}, false);
        chassis->swingToHeading(85, lemlib::DriveSide::RIGHT, 900, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    }

    {
        dt->l_chassis.reset_location_force(NEG_NEG);
    }

    {
        chassis->moveToPose(POS(coords::segment_tres::long_goal_neg_neg), 900, {}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(2000);
        dt->l_chassis.reset_location_force(NEG_NEG);
    }

    {
        conv->match_loader.toggle();
        (void)conv->exhaust.move(-FULL_POWER * 0.2);
        chassis->moveToPose(POS(coords::segment_tres::match_loader_neg_neg), 1000, {.forwards = false}, false);
        chassis->tank(MATCH_LOADER, MATCH_LOADER, true);
        pros::Task::delay(200);
        brake_motors(chassis);
        pros::Task::delay(1800);
        dt->l_chassis.reset_location_force(NEG_NEG);
    }

    {
        chassis->moveToPose(POS(coords::segment_tres::long_goal_neg_neg), 1000, {}, false);
        chassis->tank(LONG_GOAL, LONG_GOAL, true);
        (void)conv->exhaust.move(FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        pros::Task::delay(2000);
        dt->l_chassis.reset_location_force(NEG_NEG);
    }

    {
        conv->match_loader.toggle();
    }

    {
        chassis->moveToPose(POS(coords::segment_tres::parking_zone_red), 1400, {.forwards = false, .lead = 0.1, .minSpeed = 40, .earlyExitRange = 0.5}, false);
        chassis->tank(-FULL_POWER, -FULL_POWER, true);
        pros::Task::delay(600);
        chassis->tank(NO_POWER, NO_POWER, true);
    }

    stop_loop = true;

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