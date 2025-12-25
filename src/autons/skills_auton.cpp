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

//#define SECTION_1
#define SECTION_2

namespace coords
{
    namespace quad_uno
    {
        pos match_loader_prime(-54.8, -50.0, 90);
        pos quadrant_trans_a(-24, -33.0, 90);
        pos quadrant_trans_b(33, -33.0, 90);
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
    using namespace coords::quad_uno;
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
        (void)conv->exhaust.move(-0.2 * FULL_POWER);
        chassis->moveToPose(-54, 46, 90, 2000, {.forwards = false, .lead = 0.3, .maxSpeed = 90}, false);
    }

    {
        dt->l_chassis.reset_location_normal(NEG_POS, NEG_POS);
    }

    {
        pros::Task::delay(2000);
        (void)conv->conveyor_intake.brake();
    }

    {
        chassis->moveToPose(-24, 56, 90, 1000, {.minSpeed = 40}, false);
        conv->match_loader.toggle();
        chassis->moveToPoint(33, 56.5, 1000, {.minSpeed = 40}, false);
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
        chassis->moveToPose(59, 46.5, 270, 1000, { .forwards = false }, false);
        chassis->tank(-20, -20, true);
        pros::Task::delay(2000);
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
        chassis->tank(40, 40, true);0
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
        (void)conv->exhaust.move(-0.2 * FULL_POWER);
        (void)conv->conveyor_intake.move(FULL_POWER);
        dt->l_chassis.reset_location_normal(POS_POS, POS_NEG);
    }

    {
        chassis->moveToPose(65, 10, 90, 1400, {.forwards = false, .lead = 0.1, .minSpeed = 40, .earlyExitRange = 0.5}, false);
        chassis->tank(-94, -90, true);
        pros::Task::delay(1100);
        conv->match_loader.toggle();
        pros::Task::delay(300);
        conv->match_loader.toggle();
    }

    {
        chassis->swingToHeading(115, lemlib::DriveSide::LEFT, 2000, {}, false);
        pros::Task::delay(500);
    }

    {
        dt->l_chassis.reset_location_force(POS_NEG);
        (void)conv->conveyor_intake.move(0.9 * FULL_POWER);
    }

    {
        chassis->moveToPose(33, -15, 35, 2000, {.forwards = false}, false);
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