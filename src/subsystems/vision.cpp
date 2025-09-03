//
// Created by aaron on 9/3/2025.
//

#include "../../include/subsystems/vision.h"
#include "../../include/ports.h"

std::unique_ptr<vision> vision_instance = nullptr;

using namespace ports::vision;

vision::vision() :
subsystem(false),
vision_front(VISION_FRONT),
vision_code(0,0,0,0,0,0,0)
{

}

void vision::tick_implementation()
{
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


