//
// Created by aaron on 7/24/2025.
//

#ifndef AUTON_H
#define AUTON_H
#include <functional>


class auton
{
    const std::function<void()> FunctionRef;
    public:
    explicit auton(const std::function<void()>& lambda) : FunctionRef(lambda) {}

    std::function<void()> Get();
};

#endif //AUTON_H
