#include "ik.h"

simxInt InverseKinematics(simxFloat x, simxFloat y, simxFloat z)
{
	float a1, a2, a3, a4, a5, a6;//六轴的角度
	float r = x * x + y * y + z * z;//绝对距离
	float f1, f2, f3, g1, g2, g3;//中间变量

	a3 = asin(1.1125 - r / 392000);//a3求解

	f1 = 350 - 560 * sin(a3);
	f2 = -560 * cos(a3);
	f3 = 0;

	a2 = asin(z / sqrt(f1 * f1 + f2 * f2)) - atan2(f2, f1);//a2求解
	if (a2 < 0)	a2 = PI - 2 * atan2(f2, f1) - a2;

	g1 = cos(a2) * f1 - sin(a2) * f2;
	g2 = -f3;
	
	a1 = acos(x / sqrt(g1 * g1 + g2 * g2)) - atan2(g2, g1);//a1求解
	

}
