#include "main.h"
#include "../include/subsystems/conveyor.hpp"
#include "../include/ports.hpp"
#include "autons.hpp"//This is needed for autons to show up
#include "lemlib/pose.hpp"
#include "titanselect/titanselect.hpp"
#include <fstream>

#include "lemlib/pid.hpp"
#include "subsystems/localization.hpp"
#include "../include/autons.hpp"

extern "C"
{
	#include "titanselect/titanselect.h"
}

#define COMPETITION

//For compile_commands.json to be configured, run: pros build-compile-commands

localization* odom = nullptr;
conveyor* conv = nullptr;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	ts_display_selector();
	odom = localization::get();
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
		lemlib::Pose pose = odom->lem_chassis.getPose();
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

void pid_tune_mode()
{
	controller_master.clear();
	pros::delay(50);

	bool is_lateral = false;//Angular mode is 0, lateral mode is 1.
	bool first_run = true;

	lemlib::PID* activePID = &odom->lem_chassis.angularPID;
	lemlib::Chassis* chassis = &odom->lem_chassis;

	while (true)
	{
		if (first_run)
		{
			controller_master.clear();
			pros::delay(50);
			first_run = false;
			continue;
		}


		lemlib::Pose pose = chassis->getPose();

		controller_master.print(0, 0, "PID MODE: %d", is_lateral);
		pros::delay(50);
		controller_master.print(1, 0, "KP:%.2f, KD:%.2f", activePID->kP, activePID->kD);
		pros::delay(50);
		controller_master.print(2, 0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
		pros::delay(50);

		if (controller_master.get_digital_new_press(ports::tune::SWAP_MODES))
		{
			if (is_lateral == false)
			{
				is_lateral = true;
				activePID = &odom->lem_chassis.lateralPID;
			}
			else
			{
				is_lateral = false;
				activePID = &odom->lem_chassis.angularPID;
			}
		}

		if (controller_master.get_digital(ports::tune::KP_UP))
		{
			activePID->kP += 0.01f;
		}

		if (controller_master.get_digital(ports::tune::KP_DOWN))
		{
			activePID->kP -= 0.01f;
		}

		if (controller_master.get_digital(ports::tune::KD_UP))
		{
			activePID->kD += 0.01f;
		}

		if (controller_master.get_digital(ports::tune::KD_DOWN))
		{
			activePID->kD -= 0.01f;
		}

		if (controller_master.get_digital_new_press(ports::tune::TEST_ANGULAR))
		{
			chassis->setPose(0,0,0);
			chassis->turnToHeading(180, 3000);
		}

		if (controller_master.get_digital_new_press(ports::tune::TEST_LATERAL))
		{
			chassis->setPose(0,0,0);
			chassis->moveToPoint(0, 40, 4000);
		}

		if (false)
		{
			std::ofstream output("pid_values.txt");

			output << "Angular:" << std::endl;
			output << "KP: " << chassis->angularPID.kP << " KI: " << chassis->angularPID.kI << "KD: " << chassis->angularPID.kD << std::endl;
			output << "Lateral:" << std::endl;
			output << "KP: " << chassis->lateralPID.kP << " KI: " << chassis->lateralPID.kI << "KD: " << chassis->lateralPID.kD << std::endl;

			output.close();

			break;
		}

		pros::delay(50);
	}
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

constexpr bool RUN_AUTO_DRIVER = false;

void opcontrol() {
	if (RUN_AUTO_DRIVER)
	{
		while (true)
		{
			if (controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
			{
				break;
			}
			pros::Task::delay(20);
		}
		auto bruh = new pros::Task ([]()
		{
			skills_routine();
		});
		while (true)
		{
			if (controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
			{
				bruh->remove();
				break;
			}

			pros::Task::delay(20);
		}

	}


    odom = localization::get();
    conv = conveyor::get();
	ts::selector* sel = ts::selector::get();

	odom->lem_chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	//odom->l_chassis.start_location_recording("ACEEL");

#ifndef COMPETITION
	if (controller_master.get_digital(ports::tune::PID_TUNE_MODE))
	{
		pid_tune_mode();
	}
#endif
	if (pros::c::competition_get_status() & COMPETITION_CONNECTED)
	{
		//conv->wings.extend();
	}

	//odom->l_chassis.perform_dsr_init(POS_POS, 270);

	std::string auton_name = sel->get_selected_auton_name();
	while (true) {
		if(odom->motors_left.is_over_temp() || odom->motors_right.is_over_temp())
		{
			controller_master.print(1, 0, "DT MOTORS HOT");
		} else
		{
			auton_name = sel->get_selected_auton_name();
#ifdef COMPETITION
			controller_master.print(1, 0, "TSA: %s          ", auton_name.c_str());
#else
			//odom->l_chassis.perform_dsr();
			lemlib::Pose pose = odom->lem_chassis.getPose();//l_chassis.get_position_calculation(odom->l_chassis.get_quadrant()).get_value();
			controller_master.print(1,0, "%.2f, %.2f, %.2f           ", pose.x, pose.y, pose.theta);
			//std::cout << pose.x << ", " << pose.y << ", " << pose.theta << std::endl;
#endif
		}

        if(controller_master.get_digital_new_press(ports::CYCLE_AUTONS))
        {
            sel->cycle_autons();
        	controller_master.rumble("..");
        	pros::delay(100);
        }

#ifndef COMPETITION
        if(controller_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
        {
        	odom->l_chassis.perform_dsr_init(NEG_NEG, 270);
        	//(void)conv->conveyor_intake.move(-90);
        	//pros::Task::delay(200);
        	//conv->do_low_goal_macro = true;
        	//pros::Task::delay(3800);
        	//conv->do_low_goal_macro = false;
        }
#endif

		lv_timer_handler();
        odom->tick();
        conv->tick();

		pros::delay(20);                               // Run for 20 ms then update
	}
}