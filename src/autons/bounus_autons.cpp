//
// Created by aaron on 10/22/2025.
//

#include "../../include/autons.h"
#include "../../include/titanselect/titanselect.hpp"
#include "../../include/subsystems/drivetrain.h"
#include "../../include/subsystems/conveyor.h"
#include "../../include/pros/adi.hpp"
#include "../../include/pros/misc.hpp"
#include "../../include/pros/rtos.hpp"
#include "../../include/pros/motors.hpp"

constexpr auto FULL_POWER = 127;

namespace coords
{
    //14.00, -25.26, -42.70 // Block trio
    //35.96, -43.93, -51.88 // Block duo under goal
    //28.00, -13.00, 0 // Primer location before scoring coming out of under goal
    //43.71, -24.50, -180 // Long Goal (prime) (right infront of)
    //42.50, 4.90, -180 // Match Loader

    namespace left
    {
        pos block_trio(14.00, -25.26, -42.70);
        pos block_duo(35.96, -43.93, -51.88);
        pos primer_score(28.00, -13.00, 0);
        pos long_goal_prime(43.71, -24.50, -180.0);
        pos match_loader_prime(42.50, 4.90, -180.0);
    }

    //-14.00, -25.26, 42.70 // Block trio
    //-35.96, -43.93, 51.88 // Block duo under goal
    //-28.00, -13.00, 0 // Primer location before scoring coming out of under goal
    //-43.71 -24.50, 180 // Long Goal (prime) (right infront of)
    //-42.50, 4.90, 180 // Match Loader

    namespace right
    {
        pos block_trio(-14.00, -25.26, 42.70);
        pos block_duo(-35.96, -43.93, 51.88);
        pos primer_score(-28.00, -13.00, 0);
        pos long_goal_prime(-43.71, -24.50, 180.0);
        pos match_loader_prime(-42.50, 4.90, 180.0);
    }
}

void nine_left_auton()
{
    //For the position marks.
    using namespace coords::left;
    //Get drivetrain object
    drivetrain* dt  = drivetrain::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    //Set lemlib chassis object pose.
    chassis->setPose(0, 0, 0);

    {   //Start intake and conveyor
        conv->intake.move(FULL_POWER);
        conv->conveyor_group.move(FULL_POWER);
        conv->exhaust.move(-0.3 * FULL_POWER);
    }

    {   //Pick up blocks (trio) and block rush (duo under goal)
        chassis->moveToPose(POS(block_trio), 1400, {.forwards = false, .maxSpeed = 65 , .minSpeed = 30}, false);
        chassis->moveToPose(POS(block_duo), 1600, {.forwards = false, .maxSpeed = 65, .minSpeed = 30, .earlyExitRange = 1}, false);
    }

    {   //Deploy match loader for block rush and wait for deployment and to pick up blocks
        conveyor::get()->lift.toggle();
        pros::delay(500);
    }

    {   //Drive to primer location then drive to long goal prime position
        chassis->moveToPose(POS(primer_score), 1500, {.minSpeed = 30}, false);
        chassis->moveToPose(POS(long_goal_prime), 1500, {.earlyExitRange = 0.5}, true);
    }

    {   //Async anti jam block while the robot is moving to the long goal. notice the last moveToPose() command has the final parameter (the async parameter) set to true.
        conv->conveyor_group.move(-FULL_POWER);
        pros::delay(600);
    }

    {   //Sync movement and stop anti jam
        conv->conveyor_group.brake();
        chassis->waitUntilDone();
    }

    {   //Tank to apply pressure while in scoring position and enable scoring with the conveyor, intake and exhaust. Wait for 2600 for scoring
        chassis->tank(30,30, true);
        conv->conveyor_group.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
        conv->intake.move(FULL_POWER);
        pros::delay(2600);
    }

    {   //Stop all elements before moving to match loader
        conv->conveyor_group.brake();
        conv->exhaust.brake();
        conv->intake.brake();
    }

    {   //Move to match loader prime then tank into to match loader free blocks
        chassis->moveToPose(POS(match_loader_prime), 1500, {.forwards = false, .minSpeed = 50}, false);
        chassis->tank(-55,-55, true);
    }

    {   //Set element manipulators to move to pick up blocks from match loader and allow in-taking for 1 second
        conv->intake.move(FULL_POWER);
        conv->conveyor_group.move(FULL_POWER);
        conv->exhaust.move(-0.3 * FULL_POWER);
        pros::delay(1000);
    }

    {   //Move to long goal prime and tank into scoring position.
        chassis->moveToPose(POS(long_goal_prime), 1500, {.earlyExitRange = 0.5}, false);
        chassis->tank(30,30, true);
    }

    {   //Set element manipulator to scoring and allow 1.4 seconds of scoring
        conv->conveyor_group.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
        conv->intake.move(FULL_POWER);
        pros::delay(1400);
    }

    {   //Stop element manipulators
        conv->conveyor_group.brake();
        conv->exhaust.brake();
        conv->intake.brake();
    }

    {   //Reverse then ram blocks
        chassis->tank(-80,-80, true);
        pros::delay(400);
        chassis->tank(100, 100, true);
    }
}

void nine_right_auton()
{
    using namespace coords::right;

    //Get drivetrain object
    drivetrain* dt  = drivetrain::get();

    //Get conveyor object
    conveyor* conv = conveyor::get();

    //Get lemlib chassis object
    lemlib::Chassis* chassis = &dt->lem_chassis;

    //Set lemlib chassis object pose.
    chassis->setPose(0, 0, 0);

    {   //Start intake and conveyor
        conv->intake.move(FULL_POWER);
        conv->conveyor_group.move(FULL_POWER);
        conv->exhaust.move(-0.3 * FULL_POWER);
    }

    {   //Pick up blocks (trio) and block rush (duo under goal)
        chassis->moveToPose(POS(block_trio), 1400, {.forwards = false, .maxSpeed = 65 , .minSpeed = 30}, false);
        chassis->moveToPose(POS(block_duo), 1600, {.forwards = false, .maxSpeed = 65, .minSpeed = 30, .earlyExitRange = 1}, false);
    }

    {   //Deploy match loader for block rush and wait for deployment and to pick up blocks
        conveyor::get()->lift.toggle();
        pros::delay(500);
    }

    {   //Drive to primer location then drive to long goal prime position
        chassis->moveToPose(POS(primer_score), 1500, {.minSpeed = 30}, false);
        chassis->moveToPose(POS(long_goal_prime), 1500, {.earlyExitRange = 0.5}, true);
    }

    {   //Async anti jam block while the robot is moving to the long goal. notice the last moveToPose() command has the final parameter (the async parameter) set to true.
        conv->conveyor_group.move(-FULL_POWER);
        pros::delay(600);
    }

    {   //Sync movement and stop anti jam
        conv->conveyor_group.brake();
        chassis->waitUntilDone();
    }

    {   //Tank to apply pressure while in scoring position and enable scoring with the conveyor, intake and exhaust. Wait for 2600 for scoring
        chassis->tank(30,30, true);
        conv->conveyor_group.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
        conv->intake.move(FULL_POWER);
        pros::delay(2600);
    }

    {   //Stop all elements before moving to match loader
        conv->conveyor_group.brake();
        conv->exhaust.brake();
        conv->intake.brake();
    }

    {   //Move to match loader prime then tank into to match loader free blocks
        chassis->moveToPose(POS(match_loader_prime), 1500, {.forwards = false, .minSpeed = 50}, false);
        chassis->tank(-55,-55, true);
    }

    {   //Set element manipulators to move to pick up blocks from match loader and allow in-taking for 1 second
        conv->intake.move(FULL_POWER);
        conv->conveyor_group.move(FULL_POWER);
        conv->exhaust.move(-0.3 * FULL_POWER);
        pros::delay(1000);
    }

    {   //Move to long goal prime and tank into scoring position.
        chassis->moveToPose(POS(long_goal_prime), 1500, {.earlyExitRange = 0.5}, false);
        chassis->tank(30,30, true);
    }

    {   //Set element manipulator to scoring and allow 1.4 seconds of scoring
        conv->conveyor_group.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
        conv->intake.move(FULL_POWER);
        pros::delay(1400);
    }

    {   //Stop element manipulators
        conv->conveyor_group.brake();
        conv->exhaust.brake();
        conv->intake.brake();
    }

    {   //Reverse then ram blocks
        chassis->tank(-80,-80, true);
        pros::delay(400);
        chassis->tank(100, 100, true);
    }
}

ts::auton autons::nine_left = ts::auton("9 Left", nine_left_auton);
ts::auton autons::nine_right = ts::auton("9 Right", nine_right_auton);
