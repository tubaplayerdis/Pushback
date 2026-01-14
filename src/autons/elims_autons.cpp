//
// Created by aaron on 1/14/2026.
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
    namespace elims
    {
        namespace left
        {

        }

        namespace right
        {

        }
    }
}

void elims_left_auton()
{

}


void elims_right_auton()
{

}

ts::auton autons::elims_left = ts::auton("ELIM LEFT", elims_left_auton);
ts::auton autons::elims_right = ts::auton("ELIM RIGHT", elims_left_auton);