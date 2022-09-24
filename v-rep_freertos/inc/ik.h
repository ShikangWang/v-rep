#ifndef __IK_H
#define __IK_H

#include "main.h"

#define A2	350
#define D4	560

typedef struct
{
	simxFloat x;
	simxFloat y;
	simxFloat z;
	simxFloat R0D[3][3];//目标点相对于坐标系0的姿态矩阵
}position_typedef;

simxInt InverseKinematics(position_typedef position, simxFloat a[6]);
static void MartixPlus(simxFloat R1[][3], simxFloat R2[][3], simxFloat DstR[][3], const simxInt width);
simxInt ArmPositionCtrl(position_typedef position);
void MovePath(position_typedef src, position_typedef dst, simxInt t);

#endif
