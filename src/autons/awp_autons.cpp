//
// Created by aaron on 10/22/2025.
//

#include "../../include/general_autons.h"
#include "../../include/titanselect/titanselect.hpp"
#include "../../include/subsystems/drivetrain.h"
#include "../../include/subsystems/conveyor.h"
#include "../../include/pros/adi.hpp"
#include "../../include/pros/misc.hpp"
#include "../../include/pros/rtos.hpp"
#include "../../include/pros/motors.hpp"

void nine_awp_low_auton()
{

}

void nine_awp_high_auton()
{

}


ts::auton autons::nine_awp_low = ts::auton("9 AWP Low", nine_awp_low_auton);
ts::auton autons::nine_awp_high = ts::auton("9 AWP High", nine_awp_high_auton());