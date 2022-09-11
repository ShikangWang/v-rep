#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"


simxInt jointVelocityCtrl(simxInt clientID, simxInt jointID, simxFloat velocity);
simxInt jointPositionCtrl(simxInt clientID, simxInt jointID, simxFloat position);


#endif

