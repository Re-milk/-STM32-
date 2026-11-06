//
//  任务划分:
//    Task_FlightControl  - 飞行控制任务 (2ms) - 最高优先级
//    Task_Sensor         - 传感器数据采集 (6ms) - 高优先级
//    Task_Communication  - 通信处理任务 (4ms) - 中高优先级
//    Task_HeightControl  - 高度控制任务 (10ms) - 中优先级
//    Task_DataReport     - 数据上报任务 (20ms) - 中低优先级
//    Task_StatusMonitor  - 状态监控任务 (50ms) - 低优先级
//    Task_Heartbeat      - 心跳任务 (1000ms) - 最低优先级

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* 应用层头文件 */
#include "ALL_DEFINE.h"
#include "ALL_DATA.h"
#include "scheduler.h"
#include "spl06.h"
#include "ANO_Data_Transfer.h"
#include "WIFI_UFO.h"
#include "flow.h"
#include "ADC.h"
#include "control.h"
#include "OLED.h"
#include "LED.h"
#include "remote.h"
#include "imu.h"
#include "mpu6050.h"
#include "nrf24l01.h"
#include "ELRS.h"
#include "mavlink_port.h"

/* 外部变量声明 - 这些变量在INIT.c中定义 */
extern _st_Mpu MPU6050;
extern _st_Mag AK8975;
extern _st_AngE Angle;
extern _st_Remote Remote;
extern _st_ALL_flag ALL_flag;
extern _st_FlightData FlightData;
extern st_Command Command;
extern PidObject pidRateX;
extern PidObject pidRateY;
extern PidObject pidRateZ;
extern PidObject pidPitch;
extern PidObject pidRoll;
extern PidObject pidYaw;
extern PidObject pidHeightRate;
extern PidObject pidHeightHigh;
extern PidObject Flow_PosPid_x;
extern PidObject Flow_PosPid_y;
extern PidObject Flow_SpeedPid_x;
extern PidObject Flow_SpeedPid_y;
extern _st_IMU IMU;
extern volatile uint32_t SysTick_count;
extern volatile uint8_t spl_flag;
extern volatile uint32_t ST_CpuID;

/* 外部函数声明 */
extern void ALL_Init(void);
extern void pid_param_Init(void);
extern void GetLockCode(void);
extern void OLED_Init(void);
extern void MpuGetData(void);
extern void FlightPidControl(float dt);
extern void MotorControl(void);
extern void GetAngle(const _st_Mpu *pMpu, _st_AngE *pAngE, float dt);
extern void ANO_NRF_Check_Event(void);
extern void ANO_DT_Data_Exchange(void);
extern void Rc_Connect(void);
extern void Mode_Controler(float dt);
extern void RC_Analy(void);
extern void Height_Get(float dt);
extern void High_Data_Calc(u8 dT_ms);
extern void HeightPidControl(float dt);
extern void ANTO_polling(void);
extern void PilotLED(void);
extern void Flag_Check(void);
extern void Voltage_Check(void);

/* 信号强度变量 - 在各模块头文件中已声明 */
extern uint8_t NRF_SSI;
extern uint8_t NRF_SSI_CNT;
extern uint8_t WIFI_SSI;
extern uint8_t WIFI_SSI_CNT;
extern u8 Locat_SSI;
extern u8 Locat_SSI_CNT;
extern u8 Flow_SSI;
extern u8 Flow_SSI_CNT;
extern u8 Locat_Err;
extern u8 Locat_Mode;
extern u8 Flow_Err;

/*************************** 任务句柄 ***************************/
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t TaskFlightControl_Handle = NULL;
static TaskHandle_t TaskSensor_Handle = NULL;
static TaskHandle_t TaskCommunication_Handle = NULL;
static TaskHandle_t TaskHeightControl_Handle = NULL;
static TaskHandle_t TaskDataReport_Handle = NULL;
static TaskHandle_t TaskStatusMonitor_Handle = NULL;
static TaskHandle_t TaskHeartbeat_Handle = NULL;

/*************************** 任务函数声明 ***************************/
static void AppTaskCreate(void *pvParameters);
static void Task_FlightControl(void *pvParameters);
static void Task_Sensor(void *pvParameters);
static void Task_Communication(void *pvParameters);
static void Task_HeightControl(void *pvParameters);
static void Task_DataReport(void *pvParameters);
static void Task_StatusMonitor(void *pvParameters);
static void Task_Heartbeat(void *pvParameters);

static void BSP_Init(void);

/*****************************************************************
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   第一步: 初始化硬件
            第二步: 创建任务
            第三步: 启动FreeRTOS调度器
  ****************************************************************/
int main(void)
{
    BaseType_t xReturn = pdPASS;

    /* 先拉起 USART1 并发开机横幅：这一步在所有外设初始化之前，
       即使后面 MPU 校准 / NRF 初始化卡住，串口也能立刻看到固件跑起来了 */
    MAVLink_EarlyInit();

    /* 硬件初始化 */
    BSP_Init();

    /* 创建AppTaskCreate任务 */
    xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,
                          (const char*    )"AppTaskCreate",
                          (uint16_t       )128,
                          (void*          )NULL,
                          (UBaseType_t    )1,
                          (TaskHandle_t*  )&AppTaskCreate_Handle);

    if(pdPASS == xReturn)
        vTaskStartScheduler();
    else
        return -1;

    while(1);
}

/***********************************************************************
  * @ 函数名  ：AppTaskCreate
  * @ 函数说明：创建所有应用任务
  * @ 参数    ：pvParameters - 未使用
  * @ 返回值  ：无
  **********************************************************************/
static void AppTaskCreate(void *pvParameters)
{
    BaseType_t xReturn = pdPASS;
    (void)pvParameters;  /* 避免未使用参数警告 */

    taskENTER_CRITICAL();

    /* 创建飞行控制任务 - 最高优先级 2ms */
    xReturn = xTaskCreate((TaskFunction_t )Task_FlightControl,
                          (const char*    )"FlightCtrl",
                          (uint16_t       )256,
                          (void*          )NULL,
                          (UBaseType_t    )6,
                          (TaskHandle_t*  )&TaskFlightControl_Handle);

    /* 创建传感器任务 - 高优先级 6ms */
    xReturn = xTaskCreate((TaskFunction_t )Task_Sensor,
                          (const char*    )"Sensor",
                          (uint16_t       )256,
                          (void*          )NULL,
                          (UBaseType_t    )5,
                          (TaskHandle_t*  )&TaskSensor_Handle);

    /* 创建通信任务 - 中高优先级 4ms */
    xReturn = xTaskCreate((TaskFunction_t )Task_Communication,
                          (const char*    )"Comm",
                          (uint16_t       )256,
                          (void*          )NULL,
                          (UBaseType_t    )4,
                          (TaskHandle_t*  )&TaskCommunication_Handle);

    /* 创建高度控制任务 - 中优先级 10ms */
    xReturn = xTaskCreate((TaskFunction_t )Task_HeightControl,
                          (const char*    )"HeightCtrl",
                          (uint16_t       )256,
                          (void*          )NULL,
                          (UBaseType_t    )3,
                          (TaskHandle_t*  )&TaskHeightControl_Handle);

    /* 创建数据上报任务 - 中低优先级 20ms */
    xReturn = xTaskCreate((TaskFunction_t )Task_DataReport,
                          (const char*    )"DataReport",
                          (uint16_t       )128,
                          (void*          )NULL,
                          (UBaseType_t    )2,
                          (TaskHandle_t*  )&TaskDataReport_Handle);

    /* 创建状态监控任务 - 低优先级 50ms */
    xReturn = xTaskCreate((TaskFunction_t )Task_StatusMonitor,
                          (const char*    )"StatusMon",
                          (uint16_t       )128,
                          (void*          )NULL,
                          (UBaseType_t    )2,
                          (TaskHandle_t*  )&TaskStatusMonitor_Handle);

    /* 创建心跳任务 - 最低优先级 1000ms */
    xReturn = xTaskCreate((TaskFunction_t )Task_Heartbeat,
                          (const char*    )"Heartbeat",
                          (uint16_t       )64,
                          (void*          )NULL,
                          (UBaseType_t    )1,
                          (TaskHandle_t*  )&TaskHeartbeat_Handle);

    vTaskDelete(AppTaskCreate_Handle);

    taskEXIT_CRITICAL();
}

/**********************************************************************
  * @ 函数名  ：Task_FlightControl
  * @ 函数说明：飞行控制任务 - 2ms周期
  *             包含: MPU数据获取、姿态PID控制、电机控制
  * @ 参数    ：无
  * @ 返回值  ：无
  ********************************************************************/
static void Task_FlightControl(void *parameter)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 2;  // 2ms

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        MpuGetData();                    // 获取陀螺仪数据
        FlightPidControl(0.002f);        // 姿态PID控制
        MotorControl();                  // 电机控制

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**********************************************************************
  * @ 函数名  ：Task_Sensor
  * @ 函数说明：传感器任务 - 6ms周期
  *             包含: 姿态解算
  * @ 参数    ：无
  * @ 返回值  ：无
  ********************************************************************/
static void Task_Sensor(void *parameter)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 6;  // 6ms

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        GetAngle(&MPU6050, &Angle, 0.006f);  // 姿态解算

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**********************************************************************
  * @ 函数名  ：Task_Communication
  * @ 函数说明：通信任务 - 4ms周期
  *             包含: NRF信号扫描、数据交换、遥控连接、模式控制
  * @ 参数    ：无
  * @ 返回值  ：无
  ********************************************************************/
static void Task_Communication(void *parameter)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 4;  // 4ms

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        MAVLink_Update((uint32_t)xTaskGetTickCount());  // MAVLink 收发（USART1, 57600）
        ANO_NRF_Check_Event();           // 扫描2.4G信号
        ANO_DT_Data_Exchange();          // 发送飞机数据到遥控器
        Rc_Connect();                    // 遥控器连接处理
        Mode_Controler(0.004f);          // 模式控制选择处理

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**********************************************************************
  * @ 函数名  ：Task_HeightControl
  * @ 函数说明：高度控制任务 - 10ms周期
  *             包含: 遥控指令处理、高度获取、高度融合、气压高度控制
  * @ 参数    ：无
  * @ 返回值  ：无
  ********************************************************************/
static void Task_HeightControl(void *parameter)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;  // 10ms

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        RC_Analy();                      // 遥控器解析指令处理
        Height_Get(0.01f);               // 获取高度数据
        High_Data_Calc(10);              // 高度数据融合
        HeightPidControl(0.006f);        // 气压高度控制

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**********************************************************************
  * @ 函数名  ：Task_DataReport
  * @ 函数说明：数据上报任务 - 20ms周期
  *             包含: 上位机数据发送
  * @ 参数    ：无
  * @ 返回值  ：无
  ********************************************************************/
static void Task_DataReport(void *parameter)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 20;  // 20ms

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        ANTO_polling();                  // 定时3秒发飞机数据到上位机

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**********************************************************************
  * @ 函数名  ：Task_StatusMonitor
  * @ 函数说明：状态监控任务 - 50ms周期
  *             包含: LED刷新、状态标志检查、电压检测
  * @ 参数    ：无
  * @ 返回值  ：无
  ********************************************************************/
static void Task_StatusMonitor(void *parameter)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 50;  // 50ms

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        PilotLED();                      // LED刷新
        Flag_Check();                    // 飞行状态标志
        Voltage_Check();                 // 飞控电压检测

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**********************************************************************
  * @ 函数名  ：Task_Heartbeat
  * @ 函数说明：心跳任务 - 1000ms周期
  *             包含: 信号强度统计、看门狗喂狗
  * @ 参数    ：无
  * @ 返回值  ：无
  ********************************************************************/
static void Task_Heartbeat(void *parameter)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000;  // 1000ms

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        /* 信号强度统计 */
        NRF_SSI = NRF_SSI_CNT;
        NRF_SSI_CNT = 0;

        WIFI_SSI = WIFI_SSI_CNT;
        WIFI_SSI_CNT = 0;

        Locat_SSI = Locat_SSI_CNT;
        Locat_SSI_CNT = 0;

        Flow_SSI = Flow_SSI_CNT;
        Flow_SSI_CNT = 0;

        /* 检测视觉定位模块是否在线 */
        if(Locat_SSI > 10) Locat_Err = 0;
        else Locat_Mode = 0, Locat_Err = 1;

        /* 检测光流模块是否在线 */
        if(Flow_SSI > 10) Flow_Err = 0;
        else Flow_Err = 1;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/***********************************************************************
  * @ 函数名  ：BSP_Init
  * @ 函数说明：板级初始化，所有板子上的初始化都可以放到这个函数里面
  * @ 参数    ：无
  * @ 返回值  ：无
  *********************************************************************/
static void BSP_Init(void)
{
    /* STM32中断优先级设置为4，4位抢占优先级，0位响应优先级 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 获取CPU ID */
    GetLockCode();

    /* 系统初始化 */
    ALL_Init();

    /* OLED初始化 */
    OLED_Init();
}

/********************************END OF FILE****************************/
