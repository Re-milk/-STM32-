//========================================================================
//	调度器 - FreeRTOS版本
//	原调度器已被FreeRTOS任务替代
//	此文件保留Duty函数以兼容原有代码结构
//========================================================================

#include "ALL_DEFINE.h"
#include "scheduler.h"
#include "spl06.h"
#include "ANO_Data_Transfer.h"
#include "WIFI_UFO.h"
#include "flow.h"
#include "ADC.h"
#include "control.h"
#include "mpu6050.h"
#include "imu.h"
#include "remote.h"
#include "LED.h"

/*
 * FreeRTOS版本说明:
 *
 * 原始的loop_t结构体和main_loop()函数已被FreeRTOS任务替代
 * 各个Duty_xms()函数现在由独立的FreeRTOS任务直接调用
 *
 * 任务分配:
 *   Task_FlightControl (2ms)  -> MpuGetData() + FlightPidControl() + MotorControl()
 *   Task_Sensor (6ms)         -> GetAngle()
 *   Task_Communication (4ms)  -> ANO_NRF_Check_Event() + ANO_DT_Data_Exchange() + Rc_Connect() + Mode_Controler()
 *   Task_HeightControl (10ms) -> RC_Analy() + Height_Get() + High_Data_Calc() + HeightPidControl()
 *   Task_DataReport (20ms)    -> ANTO_polling()
 *   Task_StatusMonitor (50ms) -> PilotLED() + Flag_Check() + Voltage_Check()
 *   Task_Heartbeat (1000ms)   -> 信号强度统计
 *
 * 所有任务使用vTaskDelayUntil()实现精确定时
 */

/* 以下函数保留为空实现，以兼容可能引用它们的代码 */
void Duty_2ms(void) {}
void Duty_4ms(void) {}
void Duty_6ms(void) {}
void Duty_10ms(void) {}
void Duty_20ms(void) {}
void Duty_50ms(void) {}
void Duty_1000ms(void) {}

//////////////////////////end///////////////////////////////////////////
