#ifndef __MAIN_H
#define __MAIN_H

#include "stdio.h"
#include "extApi.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor.h"
#include "ik.h"

#define JOINT_REMOTE_INIT	1

#define PI	3.1415926
#define L	360	//�ֱ���ǰ�������һ���ؽڵ�z��������
#define TIMES	400 // ÿ��ƽ���˶��ĵ���(������4000)

extern simxInt clientID;
extern simxFloat angle[TIMES][6];
extern simxFloat present_angle[6];
extern simxFloat R0D_default[3][3];


void jointCtrlTask(void);
//void joint1Ctrl(void);
//void joint2Ctrl(void);
//void joint3Ctrl(void);
//void joint4Ctrl(void);
//void joint5Ctrl(void);
//void joint6Ctrl(void);

#endif
