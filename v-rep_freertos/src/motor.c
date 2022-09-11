#include "motor.h"
#include "string.h"
/*
@brief	控制关节的速度
@param	clientID : clinetID
		jointID : 关节序号
		velocity : 需要达到的速度
@return	1:设置成功 0 : 设置失败
*/
simxInt jointVelocityCtrl(simxInt clientID, simxInt jointID, simxFloat velocity)
{
	simxInt jointHandle;
	char jointName[7] = "joint";
	jointName[5] = (char)(0x30 + jointID);

	if (simxGetObjectHandle(clientID, jointName, &jointHandle, simx_opmode_oneshot) == simx_return_ok)
	{
		if (simxSetJointTargetVelocity(clientID, jointHandle, velocity*PI/180, simx_opmode_oneshot) == simx_return_ok)
		{
			return 1;
		}
	}
	return 0;
}

/*
@brief	控制关节的位置
@param	clientID : clinetID
		jointID : 关节序号
		velocity : 需要达到的位置
@return	1:设置成功 0 : 设置失败
*/
simxInt jointPositionCtrl(simxInt clientID, simxInt jointID, simxFloat position)
{
	simxInt jointHandle;
	char jointName[7] = "joint";
	jointName[5] = (char)(0x30 + jointID);
	if (simxGetObjectHandle(clientID, jointName, &jointHandle, simx_opmode_oneshot) == simx_return_ok)
	{
		if (simxSetJointTargetPosition(clientID, jointHandle, -position*PI/180, simx_opmode_oneshot) == simx_return_ok)
		{
			return 1;
		}
	}
	return 0;
}
