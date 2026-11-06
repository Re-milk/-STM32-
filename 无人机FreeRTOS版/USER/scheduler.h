#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "stm32f10x.h"

/*
 * FreeRTOS版本说明:
 * 原始的调度器(loop_t结构体和main_loop函数)已被FreeRTOS任务替代
 * 此文件保留是为了兼容性，所有任务调度由FreeRTOS管理
 *
 * 任务划分:
 *   Task_FlightControl  - 2ms  - 飞行控制(最高优先级)
 *   Task_Sensor         - 6ms  - 传感器(高优先级)
 *   Task_Communication  - 4ms  - 通信(中高优先级)
 *   Task_HeightControl  - 10ms - 高度控制(中优先级)
 *   Task_DataReport     - 20ms - 数据上报(中低优先级)
 *   Task_StatusMonitor  - 50ms - 状态监控(低优先级)
 *   Task_Heartbeat      - 1s   - 心跳(最低优先级)
 */

/* 保留函数声明以兼容原有代码 */
void Duty_2ms(void);
void Duty_4ms(void);
void Duty_6ms(void);
void Duty_10ms(void);
void Duty_20ms(void);
void Duty_50ms(void);
void Duty_1000ms(void);

#endif
