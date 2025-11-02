//
// Created by aaron on 7/23/2025.
//

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <memory>

#include "../cls/subsystem.hpp"
#include "../pros/motor_group.hpp"
#include "../pros/distance.hpp"
#include "../controller.hpp"
#include "ports.hpp"
#include "localization.hpp"

/// drivetrain class represents the physical drivetrain of the robot. inherits subsystem
class drivetrain final : public subsystem
{
    /// Friend class to allow unique_ptr to access deconstructor
    friend class std::unique_ptr<drivetrain>;

public:

    /// Left motor group. Includes 3 motors, A, B, C
    pros::MotorGroup motors_left;

    /// Right motor group. Includes 3 motors, A, B, C
    pros::MotorGroup motors_right;

    /// LemLib drivetrain object. Used in autons and the chassis object
    lemlib::Drivetrain lem_drivetrain;

    /// LemLib chassis object. Used in autons and for controller inputs
    lemlib::Chassis lem_chassis;

private:

    /// private constructor to enforce the usage of the get() function.
    /// creates the drivetrain object and initializes all the class variables.
    drivetrain();

protected:

    /// gathers controller info and relays that to the drivetrain.
    void tick_implementation() override;

public:

    /// public static accessor function to access the drivetrain singleton. creates the drivetrain object if the singleton is null.
    /// \return raw pointer to the drivetrain object.
    static drivetrain* get();
};

#endif //DRIVETRAIN_H
