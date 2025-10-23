#include "main.h"
#include "../include/subsystems/drivetrain.h"
#include "../include/subsystems/conveyor.h"
#include "../include/ports.h"
#include "autons.h"//This is needed for autons to show up
#include "titanselect/titanselect.hpp"
extern "C"
{
	#include "titanselect/titanselect.h"
}
//For compile_commands.json to be configured, run: pros build-compile-commands

odometry* odom = nullptr;
drivetrain* dt = nullptr;
conveyor* conv = nullptr;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	ts_display_selector();
	odom = odometry::get();
	dt = drivetrain::get();
	conv = conveyor::get();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	//Run monitor and re-calibration.
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
	if (!ts::selector::get()->is_auton_selected())
	{
		controller_master.rumble(".-.-.-.-");
        pros::delay(100);
	}
	while (true)
	{
		pros::delay(100);
		controller_master.print(2, 0, "TSA: %s", ts_get_selected_auton_name());
		lemlib::Pose pose = dt->lem_chassis.getPose();
		controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
	ts_run_selected_auton();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    odom = odometry::get();
    dt = drivetrain::get();
    conv = conveyor::get();

    pros::Task controller_screen_task([]() -> void {
        controller_master.clear();
    	pros::delay(200);
        while(true)
        {
            pros::delay(200);
            //controller_master.print(2, 0, "TSA: %s", ts_get_selected_auton_name());
            if(dt->motors_left.is_over_temp() || dt->motors_right.is_over_temp())
            {
                controller_master.print(1, 0, "MOTORS HOT");
            } else
            {
                lemlib::Pose pose = dt->lem_chassis.getPose();
                controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
            }
        }
    });

	while (true) {
        if(controller_master.get_digital_new_press(ports::CYCLE_AUTONS))
        {
            ts_cycle_autons();
        }

		lv_timer_handler();
        odom->tick();
        dt->tick();
        conv->tick();
		pros::delay(20);                               // Run for 20 ms then update
	}
}