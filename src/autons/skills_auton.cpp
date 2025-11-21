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
        pos match_loader_prime(-48.8, -49.3, 90);
        pos quadrant_trans_a(-25, -36.80, 90);
        pos quadrant_trans_b(33, -36.80, 90);
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
    drivetrain* dt  = drivetrain::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    chassis->setPose(0, 0, 0);
    localization::get()->distance_sensor_reset(SKILLS_INITIAL);

    {   //Move to match loader priming position with 2 position movement
        chassis->moveToPoint(MPOS(match_loader_prime), 3000, { .forwards = false}, false);
        chassis->turnToHeading(TPOS(match_loader_prime), 500, {}, false);
    }

    {   //Setup conveyor and exhaust to handle 7 blocks
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(-0.3 * FULL_POWER);
    }

    {
        conv->lift.toggle();
        pros::Task::delay(500);
        chassis->tank(MATCH_LOADER,MATCH_LOADER, true);
        pros::Task::delay(3000);
        chassis->tank(NO_POWER,NO_POWER, true);
    }

    {
        chassis->moveToPose(POS(quadrant_trans_a), 3000, {.minSpeed = 30, .earlyExitRange = 0.2}, false);
        conv->conveyor_intake.brake();
        conv->exhaust.brake();
        chassis->moveToPose(POS(quadrant_trans_b), 1500, {}, false);
    }

    while (true)
    {
        lemlib::Pose pose = chassis->getPose();
        controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
        pros::Task::delay(50);
    }
}

ts::auton autons::skills = ts::auton("Skills", skills_routine);