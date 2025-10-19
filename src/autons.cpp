//
// Created by aaron on 9/10/2025.
//

#include "../include/autons.h"
#include "../include/titanselect/titanselect.hpp"
#include "../include/subsystems/drivetrain.h"
#include "../include/subsystems/conveyor.h"
#include "../Include/pros/adi.hpp"
#include "../include/pros/rtos.h"
#include "../include/pros/motors.hpp"
#include "stdio.h"
#include "math.h"
#include <math.h>

#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/iostream"
#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/fstream"
#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/iosfwd"
#include "../../../../../pros-toolchain/usr/arm-none-eabi/include/c++/13.3.1/iosfwd"

constexpr auto FULL_POWER = 127;

float angleError(float targetDeg, float currentDeg, bool useDirection = false,
                 lemlib::AngularDirection preferredDirection = lemlib::AngularDirection::CW_CLOCKWISE)
{
    // Normalize both to [0, 360)
    float target = fmodf(targetDeg, 360.0f);
    if (target < 0) target += 360.0f;

    float current = fmodf(currentDeg, 360.0f);
    if (current < 0) current += 360.0f;

    // Compute raw difference
    float diff = target - current;

    // Wrap to [-180, 180)
    if (diff > 180.0f) diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;

    if (useDirection)
    {
        // Force the sign according to preferred turning direction
        if (preferredDirection == lemlib::AngularDirection::CW_CLOCKWISE)
            diff = fabsf(diff); // always turn CW (positive)
        else
            diff = -fabsf(diff); // always turn CCW (negative)
    }

    return diff; // positive means turn CW, negative means turn CCW
}

class SimpleTimer {
    uint32_t start;
public:
    SimpleTimer() { reset(); }
    void reset() { start = pros::millis(); }
    uint32_t elapsed() const { return pros::millis() - start; }
};

void testing_auton()
{
    std::ofstream deb = std::ofstream("/usd/deb.txt");
    lemlib::TurnToHeadingParams turnParams;
    turnParams.maxSpeed = 60;
    turnParams.minSpeed = 10;
    turnParams.direction = lemlib::AngularDirection::AUTO;
    turnParams.earlyExitRange = 0;

    drivetrain* dt = drivetrain::get();
    lemlib::Chassis* chassis = &dt->lem_chassis;

    chassis->setPose(0, 0, 0);
    //chassis->turnToHeading(90, 3000);
    chassis->moveToPose(0, 20, 0, 3000);

    while (true)
    {
        lemlib::Pose pose = chassis->getPose();
        controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
        pros::delay(100);
    }

    return;
    deb << "time_ms,pose_x,pose_y,pose_theta,target_theta,delta_theta,motor_power_left,motor_power_right\n";
    deb.flush();

    const float targetTheta = 45;
    const int timeout = 3000;
    const int sampleDelay = 5;

    SimpleTimer timer;
    SimpleTimer totalTimer;
    timer.reset();
    totalTimer.reset();

    lemlib::Pose pose = lemlib::Pose(0,0,0);
    float motorPowerL = 0;
    float motorPowerR = 0;

    chassis->requestMotionStart();
    if (!chassis->motionRunning) return;

    while (totalTimer.elapsed() < (uint32_t)timeout && chassis->motionRunning)
    {
        pose = chassis->getPose();
        float deltaTheta = angleError(targetTheta, pose.theta, false);

        static float prevError = 0;
        if ((deltaTheta > 0 && prevError < 0) || (deltaTheta < 0 && prevError > 0)) {
            // crossed target — stop quickly
            dt->motors_left.move(0);
            dt->motors_right.move(0);
            pros::delay(100);
        }
        prevError = deltaTheta;

        // Estimate PID output for debugging
        float pidOut = chassis->angularPID.update(deltaTheta);

        // Cap the output
        if (pidOut > turnParams.maxSpeed) pidOut = turnParams.maxSpeed;
        else if (pidOut < -turnParams.maxSpeed) pidOut = -turnParams.maxSpeed;

        // Apply min speed floor
        if (pidOut > 0 && pidOut < turnParams.minSpeed) pidOut = turnParams.minSpeed;
        if (pidOut < 0 && pidOut > -turnParams.minSpeed) pidOut = -turnParams.minSpeed;

        motorPowerL = pidOut;
        motorPowerR = -pidOut;

        // Log to file
        deb << totalTimer.elapsed() << ','
            << pose.x << ',' << pose.y << ',' << pose.theta << ','
            << targetTheta << ',' << deltaTheta << ','
            << motorPowerL << ',' << motorPowerR << '\n';
        deb.flush();

        // Command motors
        dt->motors_left.move(motorPowerL);
        dt->motors_right.move(motorPowerR);

        pros::delay(sampleDelay);
    }

    // Stop motors at the end
    dt->motors_left.move(0);
    dt->motors_right.move(0);

    deb << "END,final_theta=" << chassis->getPose().theta << "\n";
    deb.flush();

    chassis->endMotion();
}


/*
while (true)
{
    lemlib::Pose pose = chassis->getPose();
    controller_master.print(1,0, "%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
    pros::delay(100);
}
*/

void blue_left_auton()
{
    drivetrain* dt  = drivetrain::get();
}

void blue_right_auton()
{
    drivetrain* dt  = drivetrain::get();
}

// Definitions below
ts::auton autons::testing = ts::auton("testing", testing_auton);
ts::auton autons::blue_left = ts::auton("blue_left", blue_left_auton);
ts::auton autons::blue_right = ts::auton("blue_right", blue_right_auton);