#include "ik.h"

/*
@brief	�����˶�ѧ������Ƕ�
@param	position : ��Ҫ����λ��
		a[6] : ���ڱ�������������Ƕ�
@return	1 : �ɹ����; 2 : ������Χ
*/
simxInt InverseKinematics(position_typedef position, simxFloat a[6])
{
	//x,y,zΪ�����ύ������
	simxFloat x, y, z;
	x = position.x - L * position.R0D[0][2];
	y = position.y - L * position.R0D[1][2];
	z = position.z - L * position.R0D[2][2];
	simxFloat r = x * x + y * y + z * z;//���Ծ���
	simxFloat f1, f2, f3, g1, g2, g3;//�м����
	simxFloat R03Trans[3][3];//0from3
	simxFloat R36[3][3] = {0,0,0,0,0,0,0,0,0};//6from3

	if (r > (A2 + D4) * (A2 + D4) || r < (A2 - D4) * (A2 - D4))
		return 0;//������е�۷�Χ

	a[2] = asin((D4 * D4 + A2 * A2 - r) / (2 * A2 * D4));//a[2]���

	f1 = A2 - D4 * sin(a[2]);
	f2 = D4 * cos(a[2]);
	f3 = 0;

	a[1] = asin(z / sqrt(f1 * f1 + f2 * f2)) - atan2(f2, f1);//a[1]���
	if (a[1] < 0)
	{
		a[2] = -PI + 2 * atan2(y, x) - a[2];
		f1 = A2 - D4 * sin(a[2]);
		f2 = D4 * cos(a[2]);
		f3 = 0;

		a[1] = asin(z / sqrt(f1 * f1 + f2 * f2)) - atan2(f2, f1);//a[1]���
	}

	g1 = cos(a[1]) * f1 - sin(a[1]) * f2;
	g2 = -f3;
	
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
	
	if (abs(sin(a[4]) > 0.05))
	{
		a[5] = atan(-R36[1][1] / R36[1][0]);
		if (a[5] > PI / 2)	a[5] = PI - a[5];

		a[3] = atan(-R36[2][2] / R36[2][1]);
		if (a[3] > PI / 2)	a[3] = PI - a[3];

		a[4] = asin(R36[1][0] / cos(a[5]));
 	}
	else
	{
		a[5] = a[5];
		a[3] = atan2(-R36[0][1], -R36[2][1]) - a[5];
		if (a[3] > PI / 2)	a[3] = -2 * a[5] - a[3];
	}
	
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
@brief	���ƻ�е��ĩ�˵��λ��
@param	position : Ŀ��λ�õ�λ��
@return	1:���óɹ� 0 : λ�ó�����Ч��Χ
*/
simxInt ArmPositionCtrl(position_typedef position)
{
	//����ĽǶ�,a[0]��ʾjoint1�ĽǶȣ��Դ�����
	simxFloat angle[6];
	if (InverseKinematics(position, angle))
	{
		jointAllCtrl(angle);
		return 1;
	}
	else
	{
		printf("destination out of range\n");
		return 0;
	}
}

/*
@brief	ʹ��е��ƽ�������˶�
@param	src : ��ʼλ��; dst : Ŀ���λ��
		t : ��������ʱ��
@return	None
*/
void MovePath(position_typedef src, position_typedef dst, simxInt t)
{
	simxFloat a_src[6];
	simxFloat a_dst[6];

	simxFloat v_src[6] = {0,0,0,0,0,0};
	simxFloat v_dst[6] = {0,0,0,0,0,0};

	simxFloat angle[4000][6];

	//��������λ��1�ֱ��Ӧ������Ƕ�
	if (InverseKinematics(src, a_src) && InverseKinematics(dst, a_dst))
	{
		for (int i = 0; i < 6; i++)
		{
			a_src[i] *= 1000;
			a_dst[i] *= 1000;
		}

		for (int i = 0; i < 6; i++)
		{
			//���κ�������˶�����
			simxFloat k[4];
			k[0] = a_src[i];
			k[1] = v_src[i];
			k[2] = 3 * (a_dst[i] - a_src[i]) / (t * t) - (2 * v_src[i] + v_dst[i]) / t;
			k[3] = 2 * (a_src[i] - a_dst[i]) / (t * t * t) + (v_src[i] + v_dst[i]) / (t * t);
			printf("%f %f %f %f\n", k[0], k[1], k[2], k[3]);

			for (int j = 0; j < t; j++)
			{
				angle[j][i] = (k[0] + k[1] * j + k[2] * j * j + k[3] * j * j * j) / 1000;
			}
		}
		for (int i = 0; i < t; i++)
		{
			jointAllCtrl(angle[i]);
			printf("%f %f %f %f %f %f\n", angle[i][0], angle[i][1], angle[i][2], angle[i][3], angle[i][4], angle[i][5]);
		}
	}
	else
	{
		printf("path error\n");
	}

}
