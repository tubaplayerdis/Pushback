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

    (void)dt->inertial.set_heading(270);
    dt->lem_chassis.setPose(0,0,270);
    dt->l_chassis.reset_location_force(NEG_POS);

    lemlib::Pose pose = chassis->getPose();
    controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
    pros::Task::delay(50);

    {
        conv->exhaust.brake();
        conv->conveyor_intake.move(FULL_POWER);
    }

    {
        chassis->moveToPoint(-32.63, 17.5, 800, {.forwards = false, .maxSpeed = 50}, false);
        pros::Task::delay(200);
        conv->conveyor_intake.brake();
    }

    {
        chassis->turnToHeading(90, 1000, {.maxSpeed = 100}, false);
        conv->conveyor_intake.move(FULL_POWER);
    }
    //-15.5, 14

    {
        chassis->moveToPose(-8.5, 8.5, 135, 1500, {}, false);
        conv->exhaust.move(FULL_POWER * 0.4);
        pros::Task::delay(500);
        conv->exhaust.brake();
    }

    {
        conv->match_loader.toggle();
        conv->exhaust.move(-0.1 * FULL_POWER);
        chassis->moveToPose(-54, 46.5, 90, 2000, {.forwards = false, .lead = 0.3, .maxSpeed = 100}, false);
    }

    {
        dt->l_chassis.reset_location_normal(NEG_POS, NEG_POS);
    }

    {
        pros::Task::delay(3000);
        conv->conveyor_intake.brake();
    }

    {
        chassis->moveToPose(-24, 59, 90, 1000, {.minSpeed = 40}, false);
        chassis->moveToPoint(25, 59, 1000, {}, false);
        chassis->moveToPose(26, 47, 0, 1000, {}, false);
    }


    while (true)
    {
        lemlib::Pose pose = chassis->getPose();
        //controller_master.print(1,0, "%s", sup.c_str());
        controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
        pros::Task::delay(50);
    }
}

ts::auton autons::skills = ts::auton("Skills", skills_routine);