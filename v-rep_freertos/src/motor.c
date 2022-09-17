#include "motor.h"
#include "string.h"
/*
@brief	控制关节的速度
@param	clientID : clinetID
		jointID : 关节序号
		velocity : 需要达到的速度
@return	1:设置成功 0 : 设置失败
*/
simxInt jointVelocityCtrl(simxInt jointID, simxFloat velocity)
{
	simxInt jointHandle;
	char jointName[7] = "joint";
	jointName[5] = (char)(0x30 + jointID);

	if (simxGetObjectHandle(clientID, jointName, &jointHandle, simx_opmode_oneshot) == simx_return_ok)
	{
		if (simxSetJointTargetVelocity(clientID, jointHandle, velocity, simx_opmode_oneshot) == simx_return_ok)
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
simxInt jointPositionCtrl(simxInt jointID, simxFloat position)
{
	simxInt jointHandle;
	char jointName[7] = "joint";
	jointName[5] = (char)(0x30 + jointID);
	if (simxGetObjectHandle(clientID, jointName, &jointHandle, simx_opmode_oneshot) == simx_return_ok)
	{
		if (simxSetJointTargetPosition(clientID, jointHandle, position, simx_opmode_oneshot) == simx_return_ok)
		{
			return 1;
		}
	}
	return 0;
}
/*
@brief	获得关节控制(第一次无法直接驱动关节)
@param	None
@return	1:设置成功 0 : 设置失败
*/
simxInt jointInit(void)
{
	simxFloat initPosition[6] = {0,0,0,0,0,0};
	jointAllCtrl(initPosition);
#if JOINT_REMOTE_INIT
	jointAllCtrl(initPosition);
#endif
}

/*
@brief	一次性控制6个关节
@param	position[6] : 6个关节的目标位置
@return	1:设置成功 0 : 设置失败
*/
simxInt jointAllCtrl(simxFloat position[6])
{
	for (int i = 0; i < 6; i++)
	{
		jointPositionCtrl(i, position[i]);
	}
}
