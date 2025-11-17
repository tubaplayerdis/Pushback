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
        pos match_loader_prime(-24, 18, 180);
    }
}

namespace power_values
{
    constexpr auto FULL_POWER = 127;
    constexpr auto NO_POWER = 127;
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

    chassis->setPose(9.5, 17, 90);

    {   //Move to match loader priming position with 2 position movement
        chassis->moveToPoint(MPOS(match_loader_prime), 3000, {}, false);
        chassis->turnToHeading(TPOS(match_loader_prime), 500, {}, false);
    }

    {   //Setup conveyor and exhaust to handle 7 blocks
        conv->conveyor_intake.move(FULL_POWER);
        conv->exhaust.move(-0.3 * FULL_POWER);
    }

    {
        chassis->tank(MATCH_LOADER,MATCH_LOADER, true);
        pros::Task::delay(1000);
        chassis->tank(NO_POWER,NO_POWER, true);
    }
}

ts::auton autons::skills = ts::auton("Skills", skills_routine);