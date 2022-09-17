#include "motor.h"
#include "string.h"
/*
@brief	���ƹؽڵ��ٶ�
@param	clientID : clinetID
		jointID : �ؽ����
		velocity : ��Ҫ�ﵽ���ٶ�
@return	1:���óɹ� 0 : ����ʧ��
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
@brief	���ƹؽڵ�λ��
@param	clientID : clinetID
		jointID : �ؽ����
		velocity : ��Ҫ�ﵽ��λ��
@return	1:���óɹ� 0 : ����ʧ��
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
@brief	��ùؽڿ���(��һ���޷�ֱ�������ؽ�)
@param	None
@return	1:���óɹ� 0 : ����ʧ��
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
@brief	һ���Կ���6���ؽ�
@param	position[6] : 6���ؽڵ�Ŀ��λ��
@return	1:���óɹ� 0 : ����ʧ��
*/
simxInt jointAllCtrl(simxFloat position[6])
{
	for (int i = 0; i < 6; i++)
	{
		jointPositionCtrl(i, position[i]);
	}
}
