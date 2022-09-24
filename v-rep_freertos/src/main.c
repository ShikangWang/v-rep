#include "main.h"

int clientID;
float jointAngle[6];

int main()
{
	int Port = 3000;

	int clientID = simxStart("127.0.0.1", Port, 1, 1, 2000, 5);

	if (clientID != -1)
	{
		printf("V-rep connected.\n");
		jointInit();
		int count = 0;

		simxFloat R0D[3][3] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };//{1,0,0,0,0.8,-0.6,0,0.6,0.8};
		position_typedef dummy;
		memcpy(dummy.R0D, R0D, 9 * sizeof(simxInt));

		while (1)
		{
			count++;
			dummy.x = 350;
			dummy.y = 500 * sin(count / 4000.0);
			//if (count == 4000)	count = 0;
			dummy.z = 950;
			ArmPositionCtrl(dummy);
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
		jointPositionCtrl(1, count);
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
		jointPositionCtrl(2, count);
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
		jointPositionCtrl(3, count);
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
		jointPositionCtrl(4, count);
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
		jointPositionCtrl(5, count);
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
		jointPositionCtrl(6, count);
		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
	}
}