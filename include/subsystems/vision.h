//
// Created by aaron on 9/3/2025.
//

#ifndef VISION_H
#define VISION_H

#include <memory>

#include "../cls/subsystem.h"
#include "../pros/ai_vision.hpp"

/// Vision needs to be manually activated and deactivated. searching runs inside tick loop.
class vision final : public subsystem
{
    friend class std::unique_ptr<vision>;

    pros::AIVision vision_front;

    pros::AIVision::Code vision_code;

private:
    vision();

protected:
    void tick_implementation() override;
    bool activate_implementation() override;
    bool deactivate_implementation() override;

public:
    void set_code(pros::AIVision::Code code);

    static vision* get();
};

#endif //VISION_H
