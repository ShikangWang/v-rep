#ifndef __MAIN_H
#define __MAIN_H

#include "stdio.h"
#include "extApi.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor.h"
#include "ik.h"

#define PI	3.1415926

extern simxInt clientID;

void joint1Ctrl(void);
void joint2Ctrl(void);
void joint3Ctrl(void);
void joint4Ctrl(void);
void joint5Ctrl(void);
void joint6Ctrl(void);

#endif
