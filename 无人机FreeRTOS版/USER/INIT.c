//========================================================================
//	系统初始化
//========================================================================

#include "ALL_DEFINE.h"
#include "ALL_DATA.h"
#include "spl06.h"
#include "LED.h"
#include "Uart1.h"
#include "USART2.h"
#include "ANO_Data_Transfer.h"
#include "ADC.h"
#include "mavlink_port.h"

volatile uint32_t SysTick_count; //系统时钟计数
volatile uint8_t spl_flag; //气压计标志
_st_Mpu MPU6050;   //MPU6050原始数据
_st_Mag AK8975;
_st_AngE Angle;    //当前姿态角值
_st_Remote Remote; //遥控通道值


volatile uint32_t ST_CpuID;


_st_ALL_flag ALL_flag; //系统标志位集合



_st_FlightData FlightData;
//飞控数据
st_Command Command;

PidObject pidRateX; //内环PID参数
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //外环PID参数
PidObject pidRoll;
PidObject pidYaw;

PidObject pidHeightRate;
PidObject pidHeightHigh;

PidObject Flow_PosPid_x;    //外环位置
PidObject Flow_PosPid_y;

PidObject Flow_SpeedPid_x;  //内环速度
PidObject Flow_SpeedPid_y;

_st_IMU IMU;

void pid_param_Init(void); //PID控制器初始化


//获取CPU唯一ID
void GetLockCode(void)
{
	ST_CpuID = *(vu32*)(0x1ffff7e8);
}

///////////////全局初始化//////////////////////////////////
void ALL_Init(void)
{
	float STBy;

	IIC_Init();             //I2C初始化

	pid_param_Init();       //PID参数初始化

	LEDInit();              //LED灯初始化

	MpuInit();              //MPU6050初始化

	//ADC初始化
	ADC1_Init();

	/* USART1 已在 main() 开头的 MAVLink_EarlyInit() 里初始化，
	   波特率只由 mavlink_port.h 的 MAV_SERIAL_BAUD 决定；
	   ANO_Uart1_Init 是幂等的，重复调用也不会冲掉已发数据 */
	MAVLink_Init();          //MAVLink 协议层复位

	if (FLY_TYPE == 2)
	{UART2_Init(115200); }      //

	USART3_Config(420000);        //ELRS串口3初始化
	Dma_init();        //ELRS遥控器DMA-USART3

	NRF24L01_init();				//2.4G遥控通信初始化

	if(spl0601_init() >=1)	 //气压计初始化
	{
		while(1)
		{
			STBy++;
			GPIOB->BSRR = GPIO_Pin_9;
			if(STBy>=100000)
			{
				spl_flag=0;
				SPL_Err = 1;
				break;
			}
		}
	}
	else
	{
		spl_flag=1;
		SPL_Err = 0;
	}

	TIM2_PWM_Config();			//2路PWM初始化
	TIM3_PWM_Config();			//2路PWM初始化
}


void pid_param_Init(void)
{

//////////////////内环速度PID///////////////////////

	pidRateX.kp = 2.0f;
	pidRateY.kp = 2.0f;
	pidRateZ.kp = 2.7f;

	pidRateX.ki = 0.0f;
	pidRateY.ki = 0.0f;
	pidRateZ.ki = 0.0f;

	pidRateX.kd = 0.10f;
	pidRateY.kd = 0.10f;
	pidRateZ.kd = 0.05f;

/////////////外环角度PID///////////////////////////

	pidPitch.kp = 5.0f;
	pidRoll.kp = 5.0f;
	pidYaw.kp = 5.0f;

	pidPitch.ki = 0.15f;   // 积分项：自动消除加速度计零偏导致的偏飞
	pidRoll.ki = 0.15f;
	pidYaw.ki = 0.0f;

	pidPitch.kd = 0.0f;
	pidRoll.kd = 0.0f;
	pidYaw.kd = 0.0f;

	// 外环角度PID积分限幅（防止积分饱和）
	pidPitch.IntegLimitHigh =  10.0f;
	pidPitch.IntegLimitLow  = -10.0f;
	pidRoll.IntegLimitHigh  =  10.0f;
	pidRoll.IntegLimitLow   = -10.0f;

	// 外环角度PID输出限幅（限制最大修正角度，单位：度/秒）
	pidPitch.OutLimitHigh =  30.0f;
	pidPitch.OutLimitLow  = -30.0f;
	pidRoll.OutLimitHigh  =  30.0f;
	pidRoll.OutLimitLow   = -30.0f;

	//内环PID参数 速度
	pidHeightRate.kp = 0.30f;
	pidHeightRate.ki = 0.00f;
	pidHeightRate.kd = 0.003f;
	//外环PID参数
	pidHeightHigh.kp = 2.0f;
	pidHeightHigh.ki = 0.00f;
	pidHeightHigh.kd = 0.00f;


/////////////////////////////////////////////////////////////////////

	//X内环速度PID参数
	Flow_SpeedPid_x.kp = 0.610f;
	Flow_SpeedPid_x.ki = 0.000f;
	Flow_SpeedPid_x.kd = 0.400f;

	//X外环位置PID参数
	Flow_PosPid_x.kp = 2.200f;
	Flow_PosPid_x.ki = 0.000f;
	Flow_PosPid_x.kd = 0.006f;

	//////////////////////////////////////////////////////////

	//Y内环速度PID参数
	Flow_SpeedPid_y.kp = 0.610f;
	Flow_SpeedPid_y.ki = 0.000f;
	Flow_SpeedPid_y.kd = 0.400f;

	//Y外环位置PID参数
	Flow_PosPid_y.kp = 2.200f;
	Flow_PosPid_y.ki = 0.00f;
	Flow_PosPid_y.kd = 0.006f;

	pitch66 = 0.0f;
	roll66 =  0.0f;

	Command.FlightMode = NORMOL;  //初始化为姿态自稳模式
}
