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
		if (simxSetJointTargetVelocity(clientID, jointHandle, velocity*PI/180, simx_opmode_oneshot) == simx_return_ok)
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
		if (simxSetJointTargetPosition(clientID, jointHandle, -position*PI/180, simx_opmode_oneshot) == simx_return_ok)
		{
			return 1;
		}
	}
	return 0;
}
