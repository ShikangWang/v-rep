#include "motor.h"
#include "string.h"

static simxInt jointHandle[6];
/*
@brief	���ƹؽڵ��ٶ�
@param	clientID : clinetID
		jointID : �ؽ����
		velocity : ��Ҫ�ﵽ���ٶ�
@return	1:���óɹ� 0 : ����ʧ��
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
@brief	���ƹؽڵ�λ��
@param	clientID : clinetID
		jointID : �ؽ����, 1-6
		velocity : ��Ҫ�ﵽ��λ��
@return	1:���óɹ� 0 : ����ʧ��
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
@brief	��ùؽڿ���(��һ���޷�ֱ�������ؽ�)
@param	None
@return	1:���óɹ� 0 : ����ʧ��
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
@brief	һ���Կ���6���ؽ�
@param	position[6] : 6���ؽڵ�Ŀ��λ��
@return	1:���óɹ� 0 : ����ʧ��
*/
simxInt jointAllCtrl(simxFloat joint_position[6])
{
	for (int i = 0; i < 6; i++)
	{
		jointPositionCtrl(i + 1, joint_position[i]);
	}
}
