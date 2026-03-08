//
// Created by aaron on 7/20/2025.
//

#ifndef CONVEYOR_H
#define CONVEYOR_H

#include "../cls/subsystem.hpp"
#include "../pros/adi.hpp"
#include "../pros/motors.hpp"
#include "../pros/motor_group.hpp"
#include "../pros/misc.h"
#include "../controller.hpp"
#include "../pros/distance.hpp"
#include "../pros/optical.hpp"

class conveyor final : public subsystem
{
    /// Friend class to allow unique_ptr to access deconstructor
    friend class std::unique_ptr<conveyor>;

public:

    /// Exhaust/scoring system
    pros::Motor exhaust;

    /// Motor that operates the conveyor and intake
    pros::Motor conveyor_intake;

    /// Pneumatics for the piston that pulls the bands down to score on the middle trough.
    pros::adi::Pneumatics trapdoor;

    /// Lift pneumatics for match loader "little will mech"
    pros::adi::Pneumatics match_loader;

    /// Pneumatics for the "wing" mechanisms
    pros::adi::Pneumatics wings;

    /// Middle goal descore pneumatics
    pros::adi::Pneumatics descore;

private:

    pros::Task low_goal_macro;

    bool do_low_goal_macro;

    /// Private constructor to enable use of get() method.
    conveyor();

protected:

    /// Custom implementation of tick. reads controller values.
    void tick_implementation() override;

public:

    /// public get accessor for singleton.
    static conveyor* get();
};

#endif //CONVEYOR_H
