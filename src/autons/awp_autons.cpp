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
        const pos block_trio(14.00, -25.26, -42.70);
        const pos block_duo(35.96, -43.93, -51.88);
        const pos primer_score(28.00, -13.00, 0);
        const pos long_goal_prime(43.71, -24.50, -180.0);
        const pos match_loader_prime(42.50, 4.90, -180.0);
        const pos high_goal(0,0,0);
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
        const pos match_loader_prime(-42.50, 4.90, 180.0);
        const pos low_goal(0,0,0);
    }
}

void nine_awp_low_auton()
{

}

void nine_awp_high_auton()
{

}


ts::auton autons::nine_awp_low = ts::auton("9 AWP Low", nine_awp_low_auton);
ts::auton autons::nine_awp_high = ts::auton("9 AWP High", nine_awp_high_auton);