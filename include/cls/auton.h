//
// Created by aaron on 7/24/2025.
//

#ifndef AUTON_H
#define AUTON_H
#include <functional>
#include "robodash/api.h"

/// <summary>
/// Defines an auton
/// </summary>
/// <param name="name">Name of the auton</param>
/// <param name="lambda">Code of the auton in brackets {}</param>
/// <returns>None</returns>
#define AUTON(name, lambda) \
class name : public auton \
{ \
name() : auton(#name ,[]()lambda) {} \
} \

//I could have created a macro that adds the lambda and name to a vector and just retuned that, but I am waiting on a need for proprietary implementations, so I made a class.
class auton
{
    static std::vector<rd::Selector::routine_t> autons;

    const std::function<void()> FunctionRef;
    const char* AutonName;

    public:
    explicit auton(const char* name, const std::function<void()>& lambda) : FunctionRef(lambda), AutonName(name)
    {
        autons.push_back({name, lambda});
    }

    void RunAutonManually() const;

    [[nodiscard]] const std::function<void()>* Get() const;

    static std::vector<rd::Selector::routine_t> GetAutons();
};

inline void auton::RunAutonManually() const
{
    FunctionRef();
}

inline const std::function<void()> *auton::Get() const
{
    return &FunctionRef;
}

inline std::vector<rd::Selector::routine_t> auton::GetAutons()
{
    return autons;
}

#endif //AUTON_H
