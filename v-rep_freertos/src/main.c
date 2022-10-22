#include "main.h"

int clientID;
float jointAngle[6];

int test = 0;
int move_flag = 0;
simxFloat present_angle[6];
position_typedef dummy, src, mid, dst;
simxFloat R0D_default[3][3] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };//{1,0,0,0,0.8,-0.6,0,0.6,0.8};
int main()
{
	int Port = 3000;

	int clientID = simxStart("127.0.0.1", Port, 1, 1, 2000, 5);
	jointInit();
	if(clientID != -1)
	{
		printf("V-rep connected.\n");
		jointInit();

		memcpy(dummy.R0D, R0D_default, 9 * sizeof(simxInt));
		memcpy(src.R0D, R0D_default, 9 * sizeof(simxInt));
		memcpy(mid.R0D, R0D_default, 9 * sizeof(simxInt));
		memcpy(dst.R0D, R0D_default, 9 * sizeof(simxInt));
		src.x = 600;
		src.y = -500;
		src.z = 900;

		mid.x = 500;
		mid.y = 500;
		mid.z = 700;

		dst.x = -600;
		dst.y = 500;
		dst.z = 900;

		dummy.x = 550;
		dummy.y = 400;
		dummy.z = 900;

		xTaskCreate(jointCtrlTask, "jointCtrlTask", 1000, NULL, 3, NULL);
		vTaskStartScheduler();
		printf("Start scheduler\n");
		while (1)
		{
			//ArmPositionCtrl(dummy);

			//用于测试机械臂整体控制
			//count++;
			//dummy.x = 350;
			//dummy.y = 500 * sin(count / 4000.0);
			////if (count == 4000)	count = 0;
			//dummy.z = 950;
			//ArmPositionCtrl(dummy);
		}

		//simxFinish(clientID);
	}
	else {
		printf("V-rep can't be connected.");
		//extApi_sleepMs(300);
	}
	

	return 0;
}


void jointCtrlTask(void)
{
	int count = 0;
	while (1)
	{
		switch (move_flag)
		{
			case 0 :
			{
				count = 0;
				break;
			}
			case 1 :
			{
				jointAllCtrl(angle[count++]);
				break;
			}
			default : 
			{
				break;
			}
		}
		if (count == TIMES)
		{
			move_flag = 0;
			count = 0;
			test += 1;
			if (test == 4) test = 0;
		}
		if (test == 0)
		{
			//MovePath(src, dst);
			MultipointMove(src, mid, dst);
			move_flag = 1;
			test = 1;
		}
		else if (test == 2)
		{
			//MovePath(dst, src);
			MultipointMove(dst, mid, src);
			move_flag = 1;
			test = 3;
		}
		vTaskDelay(5);
	}
}

//void joint1Ctrl(void)
//{
//	int count = 0;
//	while (1)
//	{
//		count++;
//		printf("joint1Ctrling\n");
//		if (count == 360)	count = 0;
//		jointPositionCtrl(1, count);
//		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
//	}
//}
//
//void joint2Ctrl(void)
//{
//	int count = 0;
//	while (1)
//	{
//		count++;
//		printf("joint2Ctrling\n");
//		if (count == 90)	count = 0;
//		jointPositionCtrl(2, count);
//		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
//	}
//}
//
//void joint3Ctrl(void)
//{
//	int count = 0;
//	while (1)
//	{
//		count++;
//		printf("joint3Ctrling\n");
//		if (count == 90)	count = 0;
//		jointPositionCtrl(3, count);
//		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
//	}
//}
//
//void joint4Ctrl(void)
//{
//	int count = 0;
//	while (1)
//	{
//		count++;
//		printf("joint4Ctrling\n");
//		if (count == 90)	count = 0;
//		jointPositionCtrl(4, count);
//		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
//	}
//}
//
//void joint5Ctrl(void)
//{
//	int count = 0;
//	while (1)
//	{
//		count++;
//		printf("joint5Ctrling\n");
//		if (count == 360)	count = 0;
//		jointPositionCtrl(5, count);
//		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
//	}
//}
//
//void joint6Ctrl(void)
//{
//	int count = 0;
//	while (1)
//	{
//		count++;
//		printf("joint6Ctrling\n");
//		if (count == 90)	count = 0;
//		jointPositionCtrl(6, count);
//		vTaskDelay(50 / portTICK_PERIOD_MS);//20Hz
//	}
//}