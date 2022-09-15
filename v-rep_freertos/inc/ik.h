#ifndef __IK_H
#define __IK_H

#include "main.h"

#define A2	350
#define D4	560

simxInt InverseKinematics(simxFloat x, simxFloat y, simxFloat z, simxFloat R06[3][3]);
void MartixPlus(simxFloat R1[][3], simxFloat R2[][3], simxFloat DstR[][3], const simxInt width);

#endif
