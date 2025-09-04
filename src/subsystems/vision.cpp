//
// Created by aaron on 9/3/2025.
//

#include "../../include/subsystems/vision.h"
#include "../../include/pros/ai_vision.hpp"
#include "../../include/pros/ai_vision.h"
#include "../../include/ports.h"
#include <vector>

std::unique_ptr<vision> vision_instance = nullptr;

using namespace ports::vision;

vision::vision() :
subsystem(false),
vision_front(VISION_FRONT),
vision_code(0,0,0,0,0,0,0)
{

}

namespace constants
{
    constexpr int ABANDON_ITEM_WIDTH_THRESHOLD = 100;
    constexpr int RING_INTAKEN_WIDTH_THRESHOLD = 180;
    constexpr int BOT_SLOWDOWN_DISTANCE = 200;
    constexpr int MAX_OBJ_TO_TRACK = 3;

    namespace colors
    {

    }
}

void vision::tick_implementation()
{
    std::vector<pros::AIVision::Object> objects = vision_front.get_all_objects();
    pros::AIVision::Object* closest_object = nullptr;

    for (pros::AIVision::Object obj : objects)
    {
        if (obj.color == vision_front.get_color() && /*Compare OBJ Tag*/)
        {
            closest_object = &obj;
        }
    }

    //This should only run when the subsystem is enabled
}

vision* vision::get()
{
    if (!vision_instance) vision_instance = std::unique_ptr<vision>(new vision());
    return vision_instance.get();
}

bool vision::activate_implementation()
{
    vision_front.reset();
    vision_front.start_awb();
    return true;
}

bool vision::deactivate_implementation()
{
    vision_front.reset();
    return true;
}

void vision::set_code(pros::AIVision::Code code)
{
    vision_code = code;
    vision_front.set_code(code);
}


