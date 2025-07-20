//
// Created by aaron on 7/20/2025.
//

#ifndef MONITOR_H
#define MONITOR_H

namespace monitor
{
    void performAllChecks(/*Maybe add a lambda for callback on failure*/);

    //Individual Checks
    bool areConnectionsValid();

    bool areMotorsCool();
}

#endif //MONITOR_H
