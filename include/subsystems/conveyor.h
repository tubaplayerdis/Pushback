//
// Created by aaron on 7/20/2025.
//

#ifndef CONVEYOR_H
#define CONVEYOR_H

#include "../cls/subsystem.h"
#include "../pros/adi.hpp"
#include "../pros/motor_group.hpp"
#include "../pros/misc.h"
#include "../controller.h"
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
    NONE_TRUOUGH = 0,
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

    /// Distance sensor that determines if the tall or short trough is being scored on.
    pros::Distance distance_trough;

    /// Optical sensor that reads colors for the splitter
    pros::Optical splitter_optical;

    /// Lift pneumatics for double park system
    pros::adi::Pneumatics lift;

    /// Pros task that operates the color sorting loop.
    std::unique_ptr<pros::Task> color_sort_task;

    /// The current color colorsort is including.
    object_color color_sort_color;

private:

    /// Color sorting boolean. changing will not induce changes. use activate_color_sort() and deactiveate_color_sort()
    bool color_sort_active;

    /// Private constructor to enable use of get() method.
    conveyor();

public:

    /// Whether color sort is active.
    /// @note Returns by value to prevent modification of the color_sort_active variable
    bool is_color_sort_active();

    /// Activate color sorting. Restarts color sorting if already active
    void activate_color_sort();

    /// De-activate color sorting.
    void deactivate_color_sort();

    /// Toggles color sort. Returns whether it was tuned on or off.
    /// @note Returns by value to prevent modification of the color_sort_active variable
    bool toggle_color_sort();

    /// Which trough is detected. Used in control loop to determine output direction of the exhaust.
    /// @return Returns the through detected by value.
    trough_detection get_detected_through();

protected:

    /// Custom implementation of tick. reads controller values.
    void tick_implementation() override;

public:

    /// public get accessor for singleton.
    static conveyor* get();
};

#endif //CONVEYOR_H
