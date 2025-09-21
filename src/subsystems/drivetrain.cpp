//
// Created by aaron on 8/22/2025.
//
#include "../../include/subsystems/drivetrain.h"
#include "../../include/ports.h"

std::unique_ptr<drivetrain> drivetrain_instance;

using namespace ports::drivetrain;
using namespace ports::drivetrain::settings;

drivetrain::drivetrain() :
motors_left({LEFT_A, LEFT_B, LEFT_C}, DRIVETRAIN_MOTOR_CARTRIDGE),
motors_right({RIGHT_A, RIGHT_B, RIGHT_C}, DRIVETRAIN_MOTOR_CARTRIDGE),
lem_drivetrain(&motors_left, &motors_right, DRIVETRAIN_TRACK_WIDTH, DRIVETRAIN_WHEEL_DIAMETER, DRIVETRAIN_RPM, DRIVETRAIN_HORIZONTAL_DRIFT),
lem_chassis(lem_drivetrain, controller::controller_settings_lateral, controller::controller_settings_angular, odometry::get()->odom_sensors, &controller::expo_curve_throttle, &controller::expo_curve_steer)
{
    //Calibrate the chassis object (calibrates the inertial sensor)
    lem_chassis.calibrate();

    //Sets the "pose" (relative position) of the odometry system to zero.
    lem_chassis.setPose(0, 0, 0);//Set the local location controller to zero
}

void drivetrain::tick_implementation()
{
    //Acquire throttle and turning values
    int32_t throttle = controller_master.get_analog(ports::drivetrain::controls::VERTICAL_AXIS);
    int32_t turn = -1 * controller_master.get_analog(ports::drivetrain::controls::HORIZONTAL_AXIS);

    //Apply inputs
    lem_chassis.arcade(turn, throttle);
}

drivetrain* drivetrain::get()
{
    if (!drivetrain_instance) drivetrain_instance = std::unique_ptr<drivetrain>(new drivetrain());
    return drivetrain_instance.get();
}