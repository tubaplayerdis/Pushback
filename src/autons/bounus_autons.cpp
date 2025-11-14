//
// Created by aaron on 10/22/2025.
//

#include "../../include/autons.hpp"
#include "../../include/titanselect/titanselect.hpp"
#include "../../include/subsystems/drivetrain.hpp"
#include "../../include/subsystems/conveyor.hpp"
#include "../../include/pros/adi.hpp"
#include "../../include/pros/misc.hpp"
#include "../../include/pros/rtos.hpp"
#include "../../include/pros/motors.hpp"

constexpr auto FULL_POWER = 127;

void anti_jam_sync(conveyor* conv, int time)
{
    int i = 0;
    while (i < time/2)
    {
        if (i % 2 == 0) conv->conveyor_intake.move(-FULL_POWER);
        else conv->conveyor_intake.move(FULL_POWER);
        pros::delay(200);
        i++;
    }
    conv->conveyor_intake.brake();
}

namespace coords
{
    //14.00, -25.26, -42.70 // Block trio
    //35.96, -43.93, -51.88 // Block duo under goal
    //28.00, -13.00, 0 // Primer location before scoring coming out of under goal
    //43.71, -24.50, -180 // Long Goal (prime) (right infront of)
    //42.50, 4.90, -180 // Match Loader

    namespace left
    {
        const pos block_trio(14.30, -25.26, -45.10);
        const pos block_duo(36.47, -42.0, -57.88);
        const pos primer_score(23.25, -14.00, 0);
        const pos long_goal_prime(44.25, -24.30, -180.0);
        const pos match_loader_prime_prime(40.40, -3.00, -130.0);
        const pos match_loader_prime(43.4, 2.00, -180.0);
        const pos high_goal(4.52, -41.12, -135.918);
    }

    //-14.00, -25.26, 42.70 // Block trio
    //-35.96, -43.93, 51.88 // Block duo under goal
    //-28.00, -13.00, 0 // Primer location before scoring coming out of under goal
    //-43.71 -24.50, 180 // Long Goal (prime) (right infront of)
    //-42.50, 4.90, 180 // Match Loader

    namespace right
    {
        const pos block_trio(-14.30, -25.26, 45.10);
        const pos block_duo(-36.4, -44.00, 58.0);
        const pos primer_score(-28.20, -13.00, 0);
        const pos long_goal_prime(-44.0, -24.50, 180.0);
        const pos match_loader_prime(-42.75, 4.90, 180.0);
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

    conv->wings.toggle();

    {   //Start intake and conveyor
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(-0.3 * FULL_POWER);
    }

    {   //Pick up blocks (trio) and block rush (duo under goal)
        chassis->moveToPose(POS(block_trio), 1400, {.forwards = false, .maxSpeed = 65 , .minSpeed = 30}, false);
        chassis->moveToPose(POS(block_duo), 1600, {.forwards = false, .maxSpeed = 65, .minSpeed = 32, .earlyExitRange = 0.7}, false);
    }

    {   //Deploy match loader for block rush and wait for deployment and to pick up blocks
        pros::delay(500);
    }

    {   //Drive to primer location then drive to long goal prime position
        chassis->moveToPose(POS(primer_score), 1500, {.minSpeed = 30}, false);
        conveyor::get()->lift.toggle();
        chassis->moveToPose(POS(long_goal_prime), 1500, {.earlyExitRange = 0.4}, true);
    }

    {   //Async anti jam block while the robot is moving to the long goal. notice the last moveToPose() command has the final parameter (the async parameter) set to true.
        anti_jam_sync(conv, 6);
    }

    {   //Sync movement and stop anti jam
        conv->conveyor_intake.brake();
        chassis->waitUntilDone();
    }

    {   //Tank to apply pressure while in scoring position and enable scoring with the conveyor, intake and exhaust. Wait for 2600 for scoring
        chassis->tank(30,30, true);
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
        pros::delay(2600);
    }

    {   //Stop all elements before moving to match loader
        conv->conveyor_intake.brake();
        conv->exhaust.brake();
    }

    {   //Move to match loader prime then tank into to match loader free blocks
        chassis->moveToPose(POS(match_loader_prime), 1500, {.forwards = false, .minSpeed = 50}, false);
        chassis->tank(-55,-55, true);
    }

    {   //Set element manipulators to move to pick up blocks from match loader and allow in-taking for 1 second
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(-0.3 * FULL_POWER);
        pros::delay(700);
    }

    {   //Move to long goal prime and tank into scoring position.
        chassis->moveToPose(POS(long_goal_prime), 1500, {.earlyExitRange = 0.5}, false);
        chassis->tank(30,30, true);
    }

    {   //Set element manipulator to scoring and allow 1.4 seconds of scoring
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
        pros::delay(1700);
    }

    {   //Stop element manipulators
        conv->conveyor_intake.brake();
        conv->exhaust.brake();
    }

    {   //Reverse then ram blocks
        chassis->tank(-80,-80, true);
        pros::delay(400);
        chassis->tank(90, 90, true);
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

    conv->wings.toggle();

    //Set lemlib chassis object pose.
    chassis->setPose(0, 0, 0);

    {   //Start intake and conveyor
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(-0.3 * FULL_POWER);
    }

    {   //Pick up blocks (trio) and block rush (duo under goal)
        chassis->moveToPose(POS(block_trio), 1400, {.forwards = false, .maxSpeed = 65 , .minSpeed = 30}, false);
        chassis->moveToPose(POS(block_duo), 1600, {.forwards = false, .maxSpeed = 65, .minSpeed = 30, .earlyExitRange = 1}, false);
    }

    {   //Deploy match loader for block rush and wait for deployment and to pick up blocks
        pros::delay(500);
    }

    {   //Drive to primer location then drive to long goal prime position
        chassis->moveToPose(POS(primer_score), 1500, {.minSpeed = 30}, false);
        conveyor::get()->lift.toggle();
        chassis->moveToPose(POS(long_goal_prime), 1500, {.earlyExitRange = 0.5}, true);
    }

    {   //Async anti jam block while the robot is moving to the long goal. notice the last moveToPose() command has the final parameter (the async parameter) set to true.
        anti_jam_sync(conv, 6);
    }

    {   //Sync movement and stop anti jam
        conv->conveyor_intake.brake();
        chassis->waitUntilDone();
    }

    {   //Tank to apply pressure while in scoring position and enable scoring with the conveyor, intake and exhaust. Wait for 2600 for scoring
        chassis->tank(30,30, true);
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
        pros::delay(2600);
    }

    {   //Stop all elements before moving to match loader
        conv->conveyor_intake.brake();
        conv->exhaust.brake();
    }

    {   //Move to match loader prime then tank into to match loader free blocks
        chassis->moveToPose(POS(match_loader_prime), 1500, {.forwards = false, .minSpeed = 50}, false);
        chassis->tank(-55,-55, true);
    }

    {   //Set element manipulators to move to pick up blocks from match loader and allow in-taking for 1 second
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(-0.3 * FULL_POWER);
        pros::delay(700);
    }

    {   //Move to long goal prime and tank into scoring position.
        chassis->moveToPose(POS(long_goal_prime), 1500, {.earlyExitRange = 0.5}, false);
        chassis->tank(30,30, true);
    }

    {   //Set element manipulator to scoring and allow 1.4 seconds of scoring
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
        pros::delay(1800);
    }

    {   //Stop element manipulators
        conv->conveyor_intake.brake();
        conv->exhaust.brake();
    }

    {   //Reverse then ram blocks
        chassis->tank(-80,-80, true);
        pros::delay(400);
        chassis->tank(100, 100, true);
    }
}

ts::auton autons::nine_left = ts::auton("9 Left", nine_left_auton);
ts::auton autons::nine_right = ts::auton("9 Right", nine_right_auton);
