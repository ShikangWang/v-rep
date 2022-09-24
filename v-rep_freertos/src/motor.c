#include "motor.h"
#include "string.h"

static simxInt jointHandle[6];
/*
@brief	控制关节的速度
@param	clientID : clinetID
		jointID : 关节序号
		velocity : 需要达到的速度
@return	1:设置成功 0 : 设置失败
*/
simxInt jointVelocityCtrl(simxInt jointID, simxFloat velocity)
{
	char jointName[7] = "joint";
	jointName[5] = (char)(0x30 + jointID);

	if (simxSetJointTargetVelocity(clientID, jointHandle[jointID - 1], velocity, simx_opmode_oneshot) == simx_return_ok)
	{
		return 1;
	}
	return 0;
}

/*
@brief	控制关节的位置
@param	clientID : clinetID
		jointID : 关节序号, 1-6
		velocity : 需要达到的位置
@return	1:设置成功 0 : 设置失败
*/
simxInt jointPositionCtrl(simxInt jointID, simxFloat position)
{
	//simxInt jointHandle;
	char jointName[7] = "joint";
	jointName[5] = (char)(0x30 + jointID);

	if (simxSetJointTargetPosition(clientID, jointHandle[jointID-1], position, simx_opmode_oneshot) == simx_return_ok)
	{
		return 1;
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
	simxGetObjectHandle(clientID, "joint1", &jointHandle[0], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "joint2", &jointHandle[1], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "joint3", &jointHandle[2], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "joint4", &jointHandle[3], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "joint5", &jointHandle[4], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "joint6", &jointHandle[5], simx_opmode_blocking);

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
simxInt jointAllCtrl(simxFloat joint_position[6])
{
	for (int i = 0; i < 6; i++)
	{
		jointPositionCtrl(i + 1, joint_position[i]);
	}
}
