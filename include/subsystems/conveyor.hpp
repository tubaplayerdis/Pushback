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

enum trough_detection : uint8_t
{
    NONE_TROUGH = 0,
    HIGH_TROUGH = 1,
    LOW_TROUGH = 2,
};

class conveyor final : public subsystem
{
    /// Friend class to allow unique_ptr to access deconstructor
    friend class std::unique_ptr<conveyor>;

public:

    /// Intake motor.
    /// @note intake and conveyor could be but are not in a motor group because it is more strategic to have a split intake in auton.
    pros::Motor intake;

    /// Exhaust/scoring system
    pros::Motor exhaust;

    /// Dual motors that operate the conveyor
    pros::MotorGroup conveyor_group; //Runs the intake and other system requiring the path of movement.

    /// Optical sensor that reads colors for the splitter
    pros::Optical splitter_optical;

    /// Ramp pneumatics for the piston that pulls the bands down to score on the middle trough.
    pros::adi::Pneumatics ramp;

    /// Lift pneumatics for match loader "little will mech"
    pros::adi::Pneumatics lift;

    /// Pneumatics for the "wing" mechanisms
    pros::adi::Pneumatics wings;

    /// The current color colorsort is including.
    object_color color_sort_color;

private:

    /// Color sorting function
    void do_color_sort(bool* out_did_color_sort);

    /// Private constructor to enable use of get() method.
    conveyor();

public:

    /// Whether color sort is active.
    /// @note Returns by value to prevent modification of the color_sort_active variable
    bool is_color_sort_active();

    /// Toggles color sort. Returns whether it was tuned on or off.
    /// @note Returns by value to prevent modification of the color_sort_active variable
    bool toggle_color_sort();

protected:

    /// Custom implementation of tick. reads controller values.
    void tick_implementation() override;

public:

    /// public get accessor for singleton.
    static conveyor* get();
};

#endif //CONVEYOR_H
