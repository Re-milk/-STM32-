/**
  *********************************************************************
  * @file    main.c
  * @brief   ANO_Remoter遥控器 - FreeRTOS版本
  *          原始调度器被替换为FreeRTOS多任务架构
  *          MCU: STM32F103CB (Cortex-M3, 72MHz)
  *********************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "sysconfig.h"
#include "ANO_Param.h"
#include "ANO_Drv_Flash.h"

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"

/* 全局变量 */
uint8_t NRF_ENABLE = 0;

/* 任务句柄 */
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t Task500Hz_Handle = NULL;
static TaskHandle_t Task250Hz_Handle = NULL;
static TaskHandle_t Task100Hz_Handle = NULL;
static TaskHandle_t Task50Hz_Handle = NULL;
static TaskHandle_t Task20Hz_Handle = NULL;
static TaskHandle_t Task1Hz_Handle = NULL;

/* 任务函数声明 */
static void AppTaskCreate(void *pvParameters);
static void Task_500Hz(void *pvParameters);   // 2ms周期 - NRF通信+遥杆
static void Task_250Hz(void *pvParameters);   // 4ms周期 - 数据收发
static void Task_100Hz(void *pvParameters);   // 10ms周期 - 按键+手势
static void Task_50Hz(void *pvParameters);    // 20ms周期 - 按键扫描
static void Task_20Hz(void *pvParameters);    // 50ms周期 - OLED+对频
static void Task_1Hz(void *pvParameters);     // 1000ms周期 - 电池+帧率

static void BSP_Init(void);

/**
  * @brief  配置时钟
  */
static void RCC_Configuration(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA
                           | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
                           | RCC_APB2Periph_ADC1  | RCC_APB2Periph_AFIO
                           | RCC_APB2Periph_SPI1
                            , ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM3, ENABLE);
}

/**
  * @brief  主函数
  */
int main(void)
{
    BaseType_t xReturn = pdPASS;

    /* 硬件初始化 */
    BSP_Init();

    /* 创建AppTaskCreate任务 */
    xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,
                          (const char*    )"AppTaskCreate",
                          (uint16_t       )256,
                          (void*          )NULL,
                          (UBaseType_t    )1,
                          (TaskHandle_t*  )&AppTaskCreate_Handle);

    if (pdPASS == xReturn)
        vTaskStartScheduler();  /* 启动任务调度器 */
    else
        return -1;

    while (1);  /* 正常不会执行到这里 */
}

/**
  * @brief  硬件板级初始化
  */
static void BSP_Init(void)
{
    RCC_Configuration();                        // 配置时钟
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  // 中断优先级组4
    ANO_LED_Init();                             // 初始化LED + 蜂鸣器
    ANO_Param_READ();                           // 参数初始化
    USB_HID_Init();                             // 初始化USB
    ANO_SPI_Init();                             // 初始化NRF所用SPI

    NRF_ENABLE = ANO_NRF_Check();              // 检查NRF连接是否正常

    if (NRF_ENABLE)
    {
        ANO_NRF_Init(MODEL_TX2, ANO_Param.NRF_Channel);  // 设置频率信道
        Show.NRF_Err = 0;
    }
    else
        Show.NRF_Err = 1;

    ANO_LED_0_FLASH();      // LED闪烁
    ADC1_Init();            // 初始化ADC采样
    KEY_Init();             // 按键初始化
    OLED_Init();            // 初始化屏幕
}

/**
  * @brief  创建应用任务
  */
static void AppTaskCreate(void *pvParameters)
{
    taskENTER_CRITICAL();   // 进入临界区

    /* 创建500Hz任务 (2ms) - NRF通信+遥杆扫描 */
    xTaskCreate((TaskFunction_t )Task_500Hz,
                (const char*    )"Task_500Hz",
                (uint16_t       )128,
                (void*          )NULL,
                (UBaseType_t    )6,         // 最高优先级
                (TaskHandle_t*  )&Task500Hz_Handle);

    /* 创建250Hz任务 (4ms) - 数据收发 */
    xTaskCreate((TaskFunction_t )Task_250Hz,
                (const char*    )"Task_250Hz",
                (uint16_t       )128,
                (void*          )NULL,
                (UBaseType_t    )5,
                (TaskHandle_t*  )&Task250Hz_Handle);

    /* 创建100Hz任务 (10ms) - 按键+手势检测 */
    xTaskCreate((TaskFunction_t )Task_100Hz,
                (const char*    )"Task_100Hz",
                (uint16_t       )128,
                (void*          )NULL,
                (UBaseType_t    )4,
                (TaskHandle_t*  )&Task100Hz_Handle);

    /* 创建50Hz任务 (20ms) - 按键扫描 */
    xTaskCreate((TaskFunction_t )Task_50Hz,
                (const char*    )"Task_50Hz",
                (uint16_t       )64,
                (void*          )NULL,
                (UBaseType_t    )3,
                (TaskHandle_t*  )&Task50Hz_Handle);

    /* 创建20Hz任务 (50ms) - OLED显示+对频检测 */
    xTaskCreate((TaskFunction_t )Task_20Hz,
                (const char*    )"Task_20Hz",
                (uint16_t       )128,
                (void*          )NULL,
                (UBaseType_t    )2,
                (TaskHandle_t*  )&Task20Hz_Handle);

    /* 创建1Hz任务 (1000ms) - 电池+帧率统计 */
    xTaskCreate((TaskFunction_t )Task_1Hz,
                (const char*    )"Task_1Hz",
                (uint16_t       )128,
                (void*          )NULL,
                (UBaseType_t    )1,         // 最低优先级
                (TaskHandle_t*  )&Task1Hz_Handle);

    vTaskDelete(AppTaskCreate_Handle);  // 删除创建任务自身

    taskEXIT_CRITICAL();    // 退出临界区
}

/**
  * @brief  500Hz任务 (2ms) - NRF通信+遥杆扫描
  */
static void Task_500Hz(void *pvParameters)
{
    while (1)
    {
        ANO_NRF_Check_Event();  // 检查是否收到飞机数据

        if (NRF_Evnet)          // 如果收到数据就发到上位机
        {
            Usb_Hid_Adddata(NRF24L01_2_RXDATA, RX_LEN);
            Usb_Hid_Send();
            NRF_Evnet = 0;
        }

        ANO_Stick_Scan();       // 读取遥杆等数据

        vTaskDelay(2);          // 延时2ms
    }
}

/**
  * @brief  250Hz任务 (4ms) - 数据收发
  */
static void Task_250Hz(void *pvParameters)
{
    while (1)
    {
        if (USB_ReceiveFlg == TRUE)  // 如果收到上位机数据就发送到飞机
        {
            if (Hid_RxData[0] < 33)
                ANO_NRF_TxPacket(&(Hid_RxData[1]), Hid_RxData[0]);
            USB_ReceiveFlg = 0x00;
        }
        else
        {
            ANO_DT_Send_RCData();   // 发送控制数据给飞机
        }

        vTaskDelay(4);              // 延时4ms
    }
}

/**
  * @brief  100Hz任务 (10ms) - 按键+手势检测
  */
static void Task_100Hz(void *pvParameters)
{
    while (1)
    {
        key_function();             // 按键检测
        Gesture_Check();            // 显示屏摇杆上下选择操作检测
        SysTick_count++;            // 按键扫描延时用

        if (!Show.Connect_Succeed)  // 如果与飞机连接失败就发送遥控数据到电脑
        {
            ANO_DT_Send_RCData_To_Pc();
        }

        vTaskDelay(10);             // 延时10ms
    }
}

/**
  * @brief  50Hz任务 (20ms) - 按键扫描
  */
static void Task_50Hz(void *pvParameters)
{
    while (1)
    {
        ANO_key();                  // 按键扫描

        vTaskDelay(20);             // 延时20ms
    }
}

/**
  * @brief  20Hz任务 (50ms) - OLED显示+对频检测
  */
static void Task_20Hz(void *pvParameters)
{
    while (1)
    {
        Show_Duty();                // 屏幕显示程序
        NRF_Check_Ch();             // 自动对频检测

        if (send_flag)              // 发送设置数据到飞机
            ANO_DT_Send_Flag_To_Fly(set_temp, 0);

        vTaskDelay(50);             // 延时50ms
    }
}

/**
  * @brief  1Hz任务 (1000ms) - 电池+帧率统计
  */
static void Task_1Hz(void *pvParameters)
{
    while (1)
    {
        // 更新电压值
        Show.Battery_Rc = Rc.AUX55;
        if (Show.Battery_Rc < 345 && Show.Battery_Rc > 320)
            Show.low_power = 1;
        else
            Show.low_power = 0;

        // 计算收到飞机数据的帧率
        NRF_SSI = NRF_SSI_CNT;
        Rc.AUX6 = NRF_SSI;
        NRF_SSI_CNT = 0;

        /* 如果帧率为0标记为失联状态 */
        if (NRF_SSI == 0)
        {
            ANO_LED_blue_OFF;
            Show.Rc_num = 0;
            Show.Connect_Succeed = 0;
            Show.hardware_type = 0;
            Show.test_flag = 0;
            Show.windows = 0;
        }
        else
        {
            ANO_LED_blue_ON;
            Show.Connect_Succeed = 1;
        }

        vTaskDelay(1000);           // 延时1000ms
    }
}
