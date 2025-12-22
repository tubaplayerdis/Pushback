//
// Created by aaron on 7/20/2025.
//

#ifndef CONVEYOR_H
#define CONVEYOR_H

#include "../cls/subsystem.hpp"
#include "../pros/adi.hpp"
#include "../pros/motor_group.hpp"
#include "../pros/misc.h"
#include "../controller.hpp"
#include "../pros/distance.hpp"
#include "../pros/optical.hpp"

/// Represents the colors of the game objects.
enum object_color : uint8_t
{
    NEUTRAL = 0,
    BLUE = 1,
    RED = 2
};

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

private:

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
