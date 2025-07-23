//
// Created by aaron on 7/23/2025.
//

#ifndef CPID_H
#define CPID_H

#include "cls/pid.h"

#define CKP 0
#define CKI 1
#define CKD 2
#define CTOLERANCE 3
#define CMAX 127
#define CMIN 100
#define CTARGET 167 //Around 500rpm after gearing change on conveyor subsystem

class cpid : public pid
{
    public:
    cpid() : pid(CKP, CKI, CKD, CTARGET, CTOLERANCE, CMAX, CMIN) {}
};

#endif //CPID_H
