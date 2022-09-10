//#include "stdio.h"
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
#include "main.h"

int main()
{
	int Port = 3000;
	int PositionControlHandle;
	int joint1Handle;
	int objectHandle;
	int state = 0;
	simxFloat linev[3] = {2,2,2};
	simxFloat anglev[3] = {1,1,1};
	float joint1_position;
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
			//simxGetObjectHandle(clientID, "IRB140_manipulationSphere", &PositionControlHandle, simx_opmode_oneshot);
			//state = simxGetObjectPosition(clientID, PositionControlHandle, -1, position, simx_opmode_blocking);
			//positionmove[0] = position[0];
			//positionmove[1] = position[1] +0.01 * sin(count / 10.0);
			//positionmove[2] = position[2];
			//simxSetObjectPosition(clientID, PositionControlHandle, -1, positionmove, simx_opmode_oneshot);
			//printf("(%f,%f,%f)\r\n", position[0], position[1], position[2]);
			simxGetObjectHandle(clientID, "IRB140_joint1", &joint1Handle, simx_opmode_oneshot);
			simxGetJointPosition(clientID, joint1Handle, &joint1_position, simx_opmode_oneshot);
			simxSetJointTargetPosition(clientID, joint1Handle, joint1_position + 0.1*PI, simx_opmode_oneshot);

			/*关节速度控制*/
			//state = simxSetJointTargetVelocity(clientID, joint1Handle, 0.2 * PI, simx_opmode_oneshot);
			//simxGetObjectVelocity(clientID, joint1Handle, linev, anglev, simx_opmode_oneshot);
			//printf("(%f,%f,%f) (%f, %f, %f)\n", anglev[0], anglev[1], anglev[2], linev[0], linev[1], linev[2]);

		}

		simxFinish(clientID);
	}
	else {
		printf("V-rep can't be connected.");
		//extApi_sleepMs(300);
	}

	return 0;
}