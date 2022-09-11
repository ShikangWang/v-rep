#include "main.h"


int clientID;

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
		
		//while (simxGetConnectionId(clientID) != -1)
		{
			xTaskCreate(joint1Ctrl, "joint1Ctrl", 1000, NULL, 2, NULL);
			xTaskCreate(joint2Ctrl, "joint2Ctrl", 1000, NULL, 2, NULL);
			xTaskCreate(joint3Ctrl, "joint3Ctrl", 1000, NULL, 2, NULL);
			xTaskCreate(joint4Ctrl, "joint4Ctrl", 1000, NULL, 2, NULL);
			xTaskCreate(joint5Ctrl, "joint5Ctrl", 1000, NULL, 2, NULL);
			xTaskCreate(joint6Ctrl, "joint6Ctrl", 1000, NULL, 2, NULL);

			vTaskStartScheduler();
		}

		//simxFinish(clientID);
	}
	else {
		printf("V-rep can't be connected.");
		//extApi_sleepMs(300);
	}
	

	return 0;
}

void joint1Ctrl(void)
{
	int count = 0;
	while (1)
	{
		count++;
		printf("joint1Ctrling\n");
		if (count == 360)	count = 0;
		jointPositionCtrl(clientID, 1, count);
		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
	}
}

void joint2Ctrl(void)
{
	int count = 0;
	while (1)
	{
		count++;
		printf("joint2Ctrling\n");
		if (count == 90)	count = 0;
		jointPositionCtrl(clientID, 2, count);
		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
	}
}

void joint3Ctrl(void)
{
	int count = 0;
	while (1)
	{
		count++;
		printf("joint3Ctrling\n");
		if (count == 90)	count = 0;
		jointPositionCtrl(clientID, 3, count);
		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
	}
}

void joint4Ctrl(void)
{
	int count = 0;
	while (1)
	{
		count++;
		printf("joint4Ctrling\n");
		if (count == 90)	count = 0;
		jointPositionCtrl(clientID, 4, count);
		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
	}
}

void joint5Ctrl(void)
{
	int count = 0;
	while (1)
	{
		count++;
		printf("joint5Ctrling\n");
		if (count == 360)	count = 0;
		jointPositionCtrl(clientID, 5, count);
		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
	}
}

void joint6Ctrl(void)
{
	int count = 0;
	while (1)
	{
		count++;
		printf("joint6Ctrling\n");
		if (count == 90)	count = 0;
		jointPositionCtrl(clientID, 6, count);
		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
	}
}