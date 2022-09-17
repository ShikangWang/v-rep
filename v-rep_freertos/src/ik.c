#include "ik.h"

simxFloat a1, a2, a3, a4, a5, a6;//六轴的角度
simxInt InverseKinematics(simxFloat x, simxFloat y, simxFloat z, simxFloat R06[3][3])
{
	simxFloat r = x * x + y * y + z * z;//绝对距离
	simxFloat f1, f2, f3, g1, g2, g3;//中间变量
	simxFloat R03Trans[3][3];//0from3
	simxFloat R36[3][3] = {0,0,0,0,0,0,0,0,0};//6from3

	if (r > (A2 + D4) * (A2 + D4) || r < (A2 - D4) * (A2 - D4))
		return 0;//超出机械臂范围

	a3 = asin((D4 * D4 + A2 * A2 - r) / (2 * A2 * D4));//a3求解

	f1 = A2 - D4 * sin(a3);
	f2 = D4 * cos(a3);
	f3 = 0;

	a2 = asin(z / sqrt(f1 * f1 + f2 * f2)) - atan2(f2, f1);//a2求解
	if (a2 < 0)
	{
		a3 = -PI + 2 * atan2(y, x) - a3;
		f1 = A2 - D4 * sin(a3);
		f2 = D4 * cos(a3);
		f3 = 0;

		a2 = asin(z / sqrt(f1 * f1 + f2 * f2)) - atan2(f2, f1);//a2求解
	}

	g1 = cos(a2) * f1 - sin(a2) * f2;
	g2 = -f3;
	
	//a1 = acos(x / sqrt(g1 * g1 + g2 * g2)) - atan2(g2, g1);//a1求解
	//a1 = asin(y / sqrt(g1 * g1 + g2 * g2)) - atan2(g2, g1);//a1求解
	a1 = atan2(y, x) - atan2(g2, g1);
	
	//if (a1 < PI / 2 || a1 > PI / 2)	a1 = -2 * atan2(g2, g1) - a1;

	{
		R03Trans[0][0] = cos(a1) * cos(a2 + a3);
		R03Trans[0][1] = sin(a1) * cos(a2 + a3);
		R03Trans[0][2] = sin(a2 + a3);
		R03Trans[1][0] = -cos(a1) * sin(a2 + a3);
		R03Trans[1][1] = - sin(a1) * sin(a2 + a3);
		R03Trans[1][2] = cos(a2 + a3);
		R03Trans[2][0] = sin(a1);
		R03Trans[2][1] = -cos(a1);
		R03Trans[2][2] = 0;
	}

	MartixPlus(R03Trans, R06, R36, 3);

	a5 = acos(R36[1][2]);
	if (a5 > PI / 2)	a5 = PI - a5;
	
	if (abs(sin(a5) > 0.05))
	{
		a6 = atan(-R36[1][1] / R36[1][0]);
		if (a6 > PI / 2)	a6 = PI - a6;

		a4 = atan(-R36[2][2] / R36[2][1]);
		if (a4 > PI / 2)	a4 = PI - a4;

		a5 = asin(R36[1][0] / cos(a6));
 	}
	else
	{
		a6 = a6;
		a4 = atan2(-R36[0][1], -R36[2][1]) - a6;
		if (a4 > PI / 2)	a4 = -2 * a6 - a4;
	}
	
	return 1;
}


void MartixPlus(simxFloat R1[][3], simxFloat R2[][3], simxFloat DstR[][3], const simxInt width)
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
