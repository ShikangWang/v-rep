#include "ik.h"

/*
@brief	逆向运动学求解各轴角度
@param	position : 需要求解的位姿
		a[6] : 用于保存解算出的六轴角度
@return	1 : 成功求解; 2 : 超出范围
*/
simxInt InverseKinematics(position_typedef position, simxFloat a[6])
{
	//x,y,z为后三轴交点坐标
	simxFloat x, y, z;
	x = position.x - L * position.R0D[0][2];
	y = position.y - L * position.R0D[1][2];
	z = position.z - L * position.R0D[2][2];
	simxFloat r = x * x + y * y + z * z;//绝对距离
	simxFloat f1, f2, f3, g1, g2, g3;//中间变量
	simxFloat R03Trans[3][3];//0from3
	simxFloat R36[3][3] = {0,0,0,0,0,0,0,0,0};//6from3
	memset(a, 0, 6 * sizeof(simxFloat));

	if (r > (A2 + D4) * (A2 + D4) || r < (A2 - D4) * (A2 - D4))
		return 0;//超出机械臂范围

	a[2] = asin((D4 * D4 + A2 * A2 - r) / (2 * A2 * D4));//a[2]求解

	f1 = A2 - D4 * sin(a[2]);
	f2 = D4 * cos(a[2]);
	f3 = 0;

	a[1] = asin(z / sqrt(f1 * f1 + f2 * f2)) - atan2(f2, f1);//a[1]求解
	if (a[1] < 0)
	{
		a[2] = -PI - a[2];
		f1 = A2 - D4 * sin(a[2]);
		f2 = D4 * cos(a[2]);
		f3 = 0;
		a[1] = asin(z / sqrt(f1 * f1 + f2 * f2)) - atan2(f2, f1);//a[1]求解
	}

	g1 = cos(a[1]) * f1 - sin(a[1]) * f2;
	g2 = -f3;
	g3 = sin(a[1]) * f1 + cos(a[1]) * f2;
	
	a[0] = atan2(y, x) - atan2(g2, g1);

	{
		R03Trans[0][0] = cos(a[0]) * cos(a[1] + a[2]);
		R03Trans[0][1] = sin(a[0]) * cos(a[1] + a[2]);
		R03Trans[0][2] = sin(a[1] + a[2]);
		R03Trans[1][0] = -cos(a[0]) * sin(a[1] + a[2]);
		R03Trans[1][1] = - sin(a[0]) * sin(a[1] + a[2]);
		R03Trans[1][2] = cos(a[1] + a[2]);
		R03Trans[2][0] = sin(a[0]);
		R03Trans[2][1] = -cos(a[0]);
		R03Trans[2][2] = 0;
	}

	MartixPlus(R03Trans, position.R0D, R36, 3);

	a[4] = acos(R36[1][2]);
	if (a[4] > PI / 2)	a[4] = PI - a[4];
	
	if (abs(sin(a[4]) > 0.0001))
	{
		a[5] = atan(-R36[1][1] / R36[1][0]);
		if (a[5] > PI / 2)	a[5] = PI - a[5];

		a[4] = atan2(R36[1][0] / cos(a[5]), R36[1][2]);

		a[3] = atan2(R36[2][2] / sin(a[4]), -R36[0][2] / sin(a[4]));

 	}
	else
	{
		a[5] = a[5];
		a[3] = atan2(-R36[0][1], -R36[2][1]) - a[5];
		if (a[3] > PI / 2)	a[3] = -2 * a[5] - a[3];
	}
	
	printf("Angle : %f %f %f %f %f %f\n", a[0], a[1], a[2], a[3], a[4], a[5]);
	return 1;
}


static void MartixPlus(simxFloat R1[][3], simxFloat R2[][3], simxFloat DstR[][3], const simxInt width)
{
	for (int i = 0; i < width; i++)
	{
		for (int j = 0; j < width; j++)
		{
			DstR[i][j] = 0;
			for (int k = 0; k < width; k++)
			{
				DstR[i][j] += R1[i][k] * R2[k][j];
			}
		}
	}
}

/*
@brief	控制机械臂末端点的位姿
@param	position : 目标位置的位姿
@return	1:设置成功 0 : 位置超出有效范围
*/
simxInt ArmPositionCtrl(position_typedef position)
{
	//六轴的角度,a[0]表示joint1的角度，以此类推
	simxFloat a[6];
	if (InverseKinematics(position, a))
	{
		jointAllCtrl(a);
		return 1;
	}
	else
	{
		printf("destination out of range\n");
		return 0;
	}
}

simxFloat angle[TIMES][6];
/*
@brief	使机械臂平滑运动到指定位置
@param	src : 初始位姿; dst : 目标点位姿
@return	None
*/
void MovePath(position_typedef src, position_typedef dst)
{
	simxFloat a_src[6];
	simxFloat a_dst[6];

	simxInt t = TIMES;

	simxFloat v_src[6] = {0,0,0,0,0,0};
	simxFloat v_dst[6] = {0,0,0,0,0,0};

	//求解出两个位姿1分别对应的六轴角度
	if (InverseKinematics(src, a_src) && InverseKinematics(dst, a_dst))
	{
		for (int i = 0; i < 6; i++)
		{
			a_src[i] *= 10000;//防止K过小导致误差
			a_dst[i] *= 10000;
		}

		for (int i = 0; i < 6; i++)
		{
			//三次函数拟合运动曲线
			simxFloat k[4];
			k[0] = a_src[i];
			k[1] = v_src[i];
			k[2] = 3 * (a_dst[i] - a_src[i]) / (t * t) - (2 * v_src[i] + v_dst[i]) / t;
			k[3] = 2 * (a_src[i] - a_dst[i]) / (t * t * t) + (v_src[i] + v_dst[i]) / (t * t);
			printf(" K : %f %f %f %f\n", k[0], k[1], k[2], k[3]);

			for (int j = 0; j < TIMES; j++)
			{
				angle[j][i] = (k[0] + k[1] * j + k[2] * j * j + k[3] * j * j * j) / 10000;
			}
		}
	}
	else
	{
		printf("path error\n");
	}

}

/*
@brief	使机械臂平滑运动到指定中间点及中点
@param	src : 初始位姿; mid : 中间点位姿; dst : 目标点位姿
@return	None
*/
void MultipointMove(position_typedef src, position_typedef mid, position_typedef dst)
{
	simxFloat a_src[6];
	simxFloat a_mid[6];
	simxFloat a_dst[6];

	simxInt t = TIMES / 2;

	simxFloat v_src[6] = { 0,0,0,0,0,0 };
	simxFloat v_mid[6] = { 0,0,0,0,0,0 };
	simxFloat v_dst[6] = { 0,0,0,0,0,0 };

	//求解出两个位姿1分别对应的六轴角度
	if (InverseKinematics(src, a_src) && InverseKinematics(mid, a_mid) && InverseKinematics(dst, a_dst))
	{
		for (int i = 0; i < 6; i++)
		{
			a_src[i] *= 10000;//防止K过小导致误差
			a_mid[i] *= 10000;
			a_dst[i] *= 10000;
			v_mid[i] = (a_dst[i] - a_src[i]) / TIMES;
		}

		for (int i = 0; i < 6; i++)//从src到mid的运动路径拟合
		{
			//三次函数拟合运动曲线
			simxFloat k[4];
			k[0] = a_src[i];
			k[1] = v_src[i];
			k[2] = 3 * (a_mid[i] - a_src[i]) / (t * t) - (2 * v_src[i] + v_mid[i]) / t;
			k[3] = 2 * (a_src[i] - a_mid[i]) / (t * t * t) + (v_src[i] + v_mid[i]) / (t * t);
			printf(" K0 : %f %f %f %f\n", k[0], k[1], k[2], k[3]);

			for (int j = 0; j < t; j++)
			{
				angle[j][i] = (k[0] + k[1] * j + k[2] * j * j + k[3] * j * j * j) / 10000;
			}
		}

		for (int i = 0; i < 6; i++)//从mid到dst的运动路径拟合
		{
			//三次函数拟合运动曲线
			simxFloat k[4];
			k[0] = a_mid[i];
			k[1] = v_mid[i];
			k[2] = 3 * (a_dst[i] - a_mid[i]) / (t * t) - (2 * v_mid[i] + v_dst[i]) / t;
			k[3] = 2 * (a_mid[i] - a_dst[i]) / (t * t * t) + (v_mid[i] + v_dst[i]) / (t * t);
			printf(" K1 : %f %f %f %f\n", k[0], k[1], k[2], k[3]);

			for (int j = 0; j < t; j++)
			{
				angle[j + t][i] = (k[0] + k[1] * j + k[2] * j * j + k[3] * j * j * j) / 10000;
			}
		}
	}
	else
	{
		printf("path error\n");
	}
}
