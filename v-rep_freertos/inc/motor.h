#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"


simxInt jointVelocityCtrl(simxInt jointID, simxFloat velocity);
simxInt jointPositionCtrl(simxInt jointID, simxFloat position);
simxInt jointInit(void);
simxInt jointAllCtrl(simxFloat position[6]);


#endif

