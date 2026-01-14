//
// Created by aaron on 10/22/2025.
//

#include "../../../include/autons.hpp"
#include "../../../include/titanselect/titanselect.hpp"
#include "../../../include/subsystems/drivetrain.hpp"
#include "../../../include/subsystems/conveyor.hpp"
#include "../../../include/pros/adi.hpp"
#include "../../../include/pros/misc.hpp"
#include "../../../include/pros/rtos.hpp"
#include "../../../include/pros/motors.hpp"

constexpr auto FULL_POWER = 127;

namespace coords
{
    //14.00, -25.26, -42.70 // Block trio
    //35.96, -43.93, -51.88 // Block duo under goal
    //28.00, -13.00, 0 // Primer location before scoring coming out of under goal
    //43.71, -24.50, -180 // Long Goal (prime) (right infront of)
    //42.50, 4.90, -180 // Match Loader
    //

    namespace high
    {
        const pos block_trio(14.30, -25.26, -47.50);
        const pos block_duo(35.5, -42.0, -57.88);
        const pos primer_score(23.25, -14.00, 0);
        //long goal and match loader undershoot currently on the x axis due to a odometry wheel issue where driving to fast would cause it to slip. A max speed has been set on the match loader movement and theses need to have higher X-values.
        const pos long_goal_prime(43.0, -24.30, -180.0);
        const pos match_loader_prime(43.9, 2.00, -180.0);
        const pos high_goal(4.52, -41.12, -135.918);
    }

    //-14.00, -25.26, 42.70 // Block trio
    //-35.96, -43.93, 51.88 // Block duo under goal
    //-28.00, -13.00, 0 // Primer location before scoring coming out of under goal
    //-43.71 -24.50, 180 // Long Goal (prime) (right infront of)
    //-42.50, 4.90, 180 // Match Loader
    //

    namespace low
    {
        const pos block_trio(-14.00, -25.26, 42.70);
        const pos block_duo(-35.96, -43.93, 51.88);
        const pos primer_score(-28.00, -13.00, 0);
        const pos long_goal_prime(-43.71, -24.50, 180.0);
        const pos match_loader_prime(-40.50, 4.00, 180.0);
        const pos low_goal(-7.91, -38.67, -46.58);
    }
}

//-7.91, -38.67, -46.58 //Low goal
//3.96, -40.32, -133.918 //High goal

void nine_awp_low_auton()
{

}

void nine_awp_high_auton()
{
    //For the position marks.
    using namespace coords::high;
    //Get drivetrain object
    localization* dt  = localization::get();

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
        chassis->moveToPose(POS(block_duo), 1600, {.forwards = false, .maxSpeed = 65, .minSpeed = 32, .earlyExitRange = 0.5}, false);
    }

    {   //Deploy match loader for block rush and wait for deployment and to pick up blocks
        conveyor::get()->match_loader.toggle();
        pros::Task::delay(500);
    }

    {   //Drive to primer location then drive to long goal prime position
        chassis->moveToPose(POS(high_goal), 1500, {.maxSpeed = 70}, false);
    }

    {
        conv->exhaust.move(FULL_POWER);
        conv->trapdoor.toggle();
        pros::Task::delay(600);
        conv->trapdoor.toggle();
        conv->exhaust.move(-0.3 * FULL_POWER);
    }

    {   //Move to match loader prime then tank into to match loader free blocks
        chassis->moveToPose(POS(match_loader_prime), 2000, {.forwards = false, .lead = 0.3, .maxSpeed = 100}, false);
    }

    {   //Set element manipulators to move to pick up blocks from match loader and allow in-taking for 1 second
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(-0.3 * FULL_POWER);
    }

    {   //Move to match loader
        chassis->tank(-40,-40, true);
        pros::Task::delay(1300);
    }

    {
        chassis->moveToPose(POS(long_goal_prime), 1500, {.earlyExitRange = 0.5}, false);
    }

    {   //Set element manipulator to scoring and allow 1.4 seconds of scoring
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(FULL_POWER);
        pros::Task::delay(2500);
    }

    {   //Stop element manipulators
        conv->conveyor_intake.brake();
        conv->exhaust.brake();
    }

    {   //Reverse then ram blocks
        chassis->tank(-80,-80, true);
        pros::Task::delay(400);
        chassis->tank(90, 90, true);
    }
}

//ts::auton nine_awp_high = ts::auton("9 AWP High", nine_awp_high_auton);