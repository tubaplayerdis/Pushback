//
// Created by aaron on 8/22/2025.
//
#include "../../include/subsystems/drivetrain.h"
#include "../../include/ports.h"

namespace pid
{
    lemlib::ControllerSettings
    controller_settings_lateral(10, // proportional gain (kP)
                                  0, // integral gain (kI)
                                  3, // derivative gain (kD)
                                  3, // anti windup
                                  1, // small error range, in inches
                                  100, // small error range timeout, in milliseconds
                                  3, // large error range, in inches
                                  500, // large error range timeout, in milliseconds
                                  20 // maximum acceleration (slew)
    );

    lemlib::ControllerSettings
    controller_settings_angular(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,  // derivative gain (kD)
                       3,   // anti-windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );
}

std::unique_ptr<drivetrain> drivetrain_instance;

using namespace ports::drivetrain;
using namespace ports::drivetrain::settings;

drivetrain::drivetrain() :
motors_left({LEFT_A, LEFT_B, LEFT_C}, DRIVETRAIN_MOTOR_CARTRIDGE),
motors_right({RIGHT_A, RIGHT_B, RIGHT_C}, DRIVETRAIN_MOTOR_CARTRIDGE),
lem_drivetrain(&motors_left, &motors_right, DRIVETRAIN_TRACK_WIDTH, DRIVETRAIN_WHEEL_DIAMETER, DRIVETRAIN_RPM, DRIVETRAIN_HORIZONTAL_DRIFT),
lem_chassis(lem_drivetrain, pid::controller_settings_lateral, pid::controller_settings_angular, odometry::get()->odom_sensors, &controller::expo_curve_throttle, &controller::expo_curve_steer)
{

    //Calibrate the chassis object (inertial is calibrated in odometry)
    lem_chassis.calibrate(false);

    //Sets the "pose" (relative position) of the odometry system to zero.
    //lem_chassis.setPose(x_localization, y_localization, 0);//Set the local location controller to zero

    //This behavior has been moved to autons (referring to setting the chassis pose).
}

void drivetrain::tick_implementation()
{
    //Acquire throttle and turning values
    int32_t throttle = controller_master.get_analog(ports::drivetrain::controls::VERTICAL_AXIS);
    int32_t turn = -1 * controller_master.get_analog(ports::drivetrain::controls::HORIZONTAL_AXIS);

    //Apply inputs. I have no idea as to why turn is being interpreted as throttle and vice versa (they are swaped for some reason.), but this works.
    lem_chassis.arcade(turn, throttle);
}

drivetrain* drivetrain::get()
{
    if (!drivetrain_instance) drivetrain_instance = std::unique_ptr<drivetrain>(new drivetrain());
    return drivetrain_instance.get();
}