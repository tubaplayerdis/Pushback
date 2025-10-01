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
lem_chassis(lem_drivetrain, controller::controller_settings_lateral, controller::controller_settings_angular, odometry::get()->odom_sensors, &controller::expo_curve_throttle, &controller::expo_curve_steer),
localization_rear(offsets::REAR_X, offsets::REAR_Y, LOCAL_REAR),
localization_left(offsets::LEFT_X, offsets::LEFT_Y, LOCAL_LEFT),
localization_right(offsets::RIGHT_X, offsets::RIGHT_Y, LOCAL_RIGHT)
{
    //Acquire localization values then feed it into LemLib chassis pose.
    double x_localization = get_x_position_from_sensors();
    double y_localization = get_y_position_from_sensor();

    //Calibrate the chassis object (calibrates the inertial sensor which sets its rotation to zero.)
    lem_chassis.calibrate();

    //Sets the "pose" (relative position) of the odometry system to zero.
    lem_chassis.setPose(x_localization, y_localization, 0);//Set the local location controller to zero
}

//The robots back faces the wall to start.
double drivetrain::get_x_position_from_sensors()
{
    if (localization_left.distance_sensor.get_distance() > localization_right.distance_sensor.get_distance())
    {
        //We are on the right and as such should use the sensor
        double local_dist = localization_right.distance_sensor.get_distance() / 25.4;
        return local_dist + localization_right.x_offset;
    } else
    {
        //We are on the left.
        double local_dist = localization_left.distance_sensor.get_distance() / 25.4;
        return 144.0 - (local_dist + localization_left.x_offset);
    }
}

double drivetrain::get_y_position_from_sensor()
{
    double local_dist = localization_rear.distance_sensor.get_distance() / 25.4;
    return local_dist + localization_rear.y_offset;
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