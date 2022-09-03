﻿//#include "stdio.h"
//#include"extApi.h"
//
//int main() 
//{
//	int ID = simxStart("127.0.0.1", 19997, 1, 1, 1000, 5);
//
//	if (ID != -1) {
//		printf("susseful");
//		simxStartSimulation(ID, simx_opmode_oneshot);
//	}
//	while (1) {
//
//	}
//}
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "extApi.h"

int main()
{
	int Port = 3000;
	int PositionControlHandle;
	int joint3Handle;
	int state = 0;
	float lv[3] = {0,0,0};
	float av[3] = {0,0,0};
	float position[3];
	float positionmove[3];
	//智能制造单元控制系统 Intelligent manufacturing unit control system
	//imucs
	int clientID = simxStart("127.0.0.1", Port, 1, 1, 2000, 5);

	if (clientID != -1)
	{
		printf("V-rep connected.\n");
		int count = 0;
		//extApi_sleepMs(300);
		
		while (simxGetConnectionId(clientID) != -1)
		{
			count++;
			simxGetObjectHandle(clientID, "IRB140_manipulationSphere", &PositionControlHandle, simx_opmode_oneshot);
			state = simxGetObjectPosition(clientID, PositionControlHandle, -1, position, simx_opmode_blocking);
			positionmove[0] = position[0];
			positionmove[1] = position[1] +0.1 * sin(count / 10.0);
			positionmove[2] = position[2];
			simxSetObjectPosition(clientID, PositionControlHandle, -1, positionmove, simx_opmode_oneshot);
			printf("(%f,%f,%f)\r\n", position[0], position[1], position[2]);
			//simxGetObjectHandle(clientID, "IRB140_joint3", &joint3Handle, simx_opmode_oneshot);
			//state = simxSetJointTargetPosition(clientID, &joint3Handle, 90, simx_opmode_oneshot);
			//simxGetObjectPosition(clientID, &joint3Handle, -1, &av, simx_opmode_oneshot);
			//printf("(%f,%f,%f)\n", av[0], av[1], av[2]);
			//printf("%d\n", state);

		}

		simxFinish(clientID);
	}
	else {
		printf("V-rep can't be connected.");
		//extApi_sleepMs(300);
	}

	return 0;
}