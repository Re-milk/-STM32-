//========================================================================
//	pid驱动
//========================================================================

#include "pid.h"
#include "myMath.h"	

/**************************************************************
 *������λPID����
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void pidRest(PidObject **pid,const uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
	  	pid[i]->integ = 0;
	    pid[i]->prevError = 0;
	    pid[i]->out = 0;
		pid[i]->offset = 0;
	}
}

/**************************************************************
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 ***************************************************************/	
void pidUpdate(PidObject* pid,const float dt)
{
	 float error;
	 float deriv;
	
    error = pid->desired - pid->measured; //��ǰ�Ƕ���ʵ�ʽǶȵ����

    pid->integ += error * dt;	 //�������ۼ�ֵ

    pid->integ = LIMIT(pid->integ,pid->IntegLimitLow,pid->IntegLimitHigh); //���л����޷�

    deriv = (error - pid->prevError)/dt;  //ǰ�����������΢��

    pid->out = pid->kp * error + pid->ki * pid->integ + pid->kd * deriv;//PID���

		pid->out = LIMIT(pid->out,pid->OutLimitLow,pid->OutLimitHigh); //����޷�
		
    pid->prevError = error;  //�����ϴε����
		
}

/**************************************************************
 *  CascadePID
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt)  //����PID
{	 
	pidUpdate(pidAngE,dt);    //�ȼ����⻷
	pidRate->desired = pidAngE->out;
	pidUpdate(pidRate,dt);    //�ټ����ڻ�	
}

/*******************************END*********************************/



