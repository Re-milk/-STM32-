#ifndef _ALL_USER_DATA_H_
#define _ALL_USER_DATA_H_

#include <stdint.h>

#ifndef NULL
#define NULL 0
#endif

extern volatile uint32_t SysTick_count;
extern volatile uint8_t spl_flag;
extern volatile uint32_t ST_CpuID;

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
 const float a[3];
 const float b[3];
}Butter_Parameter;


typedef struct{
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
}_st_Mpu;


typedef struct{
	int16_t magX;
	int16_t magY;
	int16_t magZ;
}_st_Mag;


typedef struct{
	float rate;
	float height;
}High;


typedef struct{
	float roll;
	float pitch;
	float yaw;
}_st_AngE;
 typedef struct
{	
//		 struct{  //�Ƕ����� 
//			float roll;
//			float pitch;
//			float yaw;
//		}Angle;
		 struct{  //�߶�����
			float rate;
			float bara_height; 
			float ultra_height;
			float ultra_baro_height;
		}High;		 
}_st_FlightData;


typedef struct
{
	uint16_t roll;
	uint16_t pitch;
	uint16_t thr;
	uint16_t yaw;
	uint16_t AUX1;
	uint16_t AUX2;
	uint16_t AUX3;
	uint16_t AUX4;	
	uint16_t AUX5;
	uint16_t AUX6;
	uint16_t AUX7;
}_st_Remote;




typedef volatile struct
{
	float desired;     ///����
	float offset;      //
	float prevError;    // �ϴ�ƫ��
	float integ;        //�������ۼ�ֵ
	float kp;           //p����
	float ki;           //i����
	float kd;           //d����
	float IntegLimitHigh;       //< integral limit
	float IntegLimitLow;
	float measured;     ////pid������
	float out;
	float OutLimitHigh;
	float OutLimitLow;	
	float Control_OutPut;//�����������
	float Last_Control_OutPut;//�ϴο����������
	float Control_OutPut_Limit;//����޷�
		/***************************************/
	float Last_FeedBack;//�ϴη���ֵ
	float Dis_Err;//΢����
	float Dis_Error_History[5];//��ʷ΢����
	float Err_LPF;
	float Last_Err_LPF;
	float Dis_Err_LPF;

//	int8_t Err_Limit_Flag :1;//ƫ���޷���־
//	int8_t Integrate_Limit_Flag :1;//�����޷���־
//	int8_t Integrate_Separation_Flag :1;//���ַ����־		
  Butter_BufferData Control_Device_LPF_Buffer;//��������ͨ�����������	
}PidObject;


typedef volatile struct
{
	uint8_t unlock;
	uint32_t  slock_flag;
	uint8_t height_lock:1;
	uint8_t take_off:1;
	uint8_t take_down:1;

}_st_ALL_flag;



typedef volatile struct
{
	uint8_t AccOffset :1;  //У׼����
	uint8_t GyroOffset :1;
	uint8_t MagOffset :1;
	uint8_t six_acc_offset; //����У׼
	enum{ 								 //����ģʽ�л�����
			LOCK = 0x00,				//����ģʽ
			NORMOL, 		//����ģʽ		
			HEIGHT,			//����ģʽ
			Flow_POSITION,   //GPS����ģʽ
	}FlightMode; //����ģʽ
}st_Command;



extern _st_Remote Remote;
extern _st_Mpu MPU6050;
extern _st_Mag AK8975; //����������Ӵ�����
extern _st_AngE Angle;


extern _st_ALL_flag ALL_flag;


extern	PidObject pidRateX;
extern	PidObject pidRateY;
extern	PidObject pidRateZ;

extern	PidObject pidPitch;
extern	PidObject pidRoll;
extern	PidObject pidYaw;

extern	PidObject pidHeightRate;
extern	PidObject pidHeightHigh;


extern PidObject Flow_PosPid_x; 
extern PidObject Flow_PosPid_y;

extern PidObject Flow_SpeedPid_x;
extern PidObject Flow_SpeedPid_y;

extern _st_FlightData FlightData;
//�ɿ�����
extern st_Command Command;

extern int16_t roll66;           //���ʱ�������ҷ���   -������������ +������������
extern int16_t pitch66;          //���ʱ����ǰ����   -����������� +������ǰ����

void GetLockCode(void);
void pid_param_Init(void);
#endif

