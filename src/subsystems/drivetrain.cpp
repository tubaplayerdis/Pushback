//
// Created by aaron on 8/22/2025.
//
#include "../../include/subsystems/drivetrain.hpp"
#include "../../include/ports.hpp"

namespace pid
{
    // Linear/lateral movement settings
    lemlib::ControllerSettings
    controller_settings_lateral(20, // proportional gain (kP)
                                              0.01, // integral gain (kI)
                                              108, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              2, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              110 // maximum acceleration (slew)
    );

    // Angular/turning settings
    lemlib::ControllerSettings
    controller_settings_angular(2.02,  // kP — reduce a bit (was 1.6)
                                0.02,  // kI — keep off
                                11.25,  // kD — increase slightly for more damping
                                0.55,    // anti-windup
                                1,  // small error range
                                100,  // small error timeout
                                2,  // large error range
                                500,  // large error timeout
                                0     // slew rate
    );
}

std::unique_ptr<drivetrain> drivetrain_instance;

using namespace ports::drivetrain;
using namespace ports::drivetrain::settings;

drivetrain::drivetrain() :
motors_left({LEFT_A, LEFT_B, LEFT_C}, DRIVETRAIN_MOTOR_CARTRIDGE),
motors_right({RIGHT_A, RIGHT_B, RIGHT_C}, DRIVETRAIN_MOTOR_CARTRIDGE),
lem_drivetrain(&motors_left, &motors_right, DRIVETRAIN_TRACK_WIDTH, DRIVETRAIN_WHEEL_DIAMETER, DRIVETRAIN_RPM, DRIVETRAIN_HORIZONTAL_DRIFT),
lem_chassis(lem_drivetrain, pid::controller_settings_lateral, pid::controller_settings_angular, localization::get()->odom_sensors, &controller::expo_curve_throttle, &controller::expo_curve_steer)
{
    motors_left.set_zero_position_all(0);
    motors_right.set_zero_position_all(0);

    //Calibrate the chassis object
    lem_chassis.calibrate(true);

    lem_chassis.setPose(0,0,0);
    //
    //    //Sets the "pose" (relative position) of the localization system to zero.
    //lem_chassis.setPose(x_localization, y_localization, 0);//Set the local location controller to zero

    //This behavior has been moved to autons (referring to setting the chassis pose).
}

void drivetrain::tick_implementation()
{
    constexpr auto FULL_POWER = 127;

    //Acquire throttle and turning values
    int32_t throttle = -1 * controller_master.get_analog(ports::drivetrain::controls::VERTICAL_AXIS);
    int32_t turn = controller_master.get_analog(ports::drivetrain::controls::HORIZONTAL_AXIS);

    //Apply inputs.
    lem_chassis.arcade(throttle, turn);

    if (controller_master.get_digital(controls::SWING_LEFT))
    {
        lem_chassis.arcade(0, -127);
    }
    else if (controller_master.get_digital(controls::SWING_RIGHT))
    {
        lem_chassis.arcade(0, 127);
    }
}

drivetrain* drivetrain::get()
{
    if (!drivetrain_instance) drivetrain_instance = std::unique_ptr<drivetrain>(new drivetrain());
    return drivetrain_instance.get();
}