/*
 * FreeRTOS配置文件 - 适配ANO_Remoter遥控器项目
 * MCU: STM32F103CB (Cortex-M3, 72MHz, 128KB Flash, 20KB RAM)
 * 基于FreeRTOS V9.0.0移植模板修改
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f10x.h"

#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
    #include <stdint.h>
    extern uint32_t SystemCoreClock;
#endif

/*********************************************************************
 *          FreeRTOS基础配置配置选项
 ********************************************************************/
/* 1:抢占式调度, 0:协作式调度 */
#define configUSE_PREEMPTION                    1

/* 1:使用时间片调度 */
#define configUSE_TIME_SLICING                  1

/* 使用硬件优化的任务选择(CLZ指令) */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1

/* 不使用tickless低功耗模式 */
#define configUSE_TICKLESS_IDLE                 0

/* CPU内核时钟频率 */
#define configCPU_CLOCK_HZ                      (SystemCoreClock)

/* RTOS系统时钟中断频率 1000Hz = 1ms tick */
#define configTICK_RATE_HZ                      ((TickType_t)1000)

/* 最大优先级数 */
#define configMAX_PRIORITIES                    (32)

/* 空闲任务栈大小 (word) */
#define configMINIMAL_STACK_SIZE                ((unsigned short)64)

/* 任务名称最大长度 */
#define configMAX_TASK_NAME_LEN                 (16)

/* 32位tick计数 */
#define configUSE_16_BIT_TICKS                  0

/* 空闲任务让出CPU */
#define configIDLE_SHOULD_YIELD                 1

/* 不使用队列集 */
#define configUSE_QUEUE_SETS                    0

/* 使用任务通知 */
#define configUSE_TASK_NOTIFICATIONS            1

/* 使用互斥信号量 */
#define configUSE_MUTEXES                       1

/* 不使用递归互斥信号量 */
#define configUSE_RECURSIVE_MUTEXES             0

/* 不使用计数信号量 */
#define configUSE_COUNTING_SEMAPHORES           0

/* 信号量/消息队列注册表大小 */
#define configQUEUE_REGISTRY_SIZE               10

#define configUSE_APPLICATION_TASK_TAG           0

/*********************************************************************
 *          FreeRTOS动态内存配置
 ********************************************************************/
/* 支持动态内存分配 */
#define configSUPPORT_DYNAMIC_ALLOCATION        1

/* 不支持静态内存 */
#define configSUPPORT_STATIC_ALLOCATION         0

/* 系统总堆大小 (STM32F103CB仅有20KB RAM, 分配8KB给FreeRTOS) */
#define configTOTAL_HEAP_SIZE                   ((size_t)(8*1024))

/*********************************************************************
 *          FreeRTOS钩子函数配置
 ********************************************************************/
/* 不使用空闲钩子 */
#define configUSE_IDLE_HOOK                     0

/* 不使用tick钩子 */
#define configUSE_TICK_HOOK                     0

/* 不使用内存分配失败钩子 */
#define configUSE_MALLOC_FAILED_HOOK            0

/* 不使用栈溢出检测 */
#define configCHECK_FOR_STACK_OVERFLOW          0

/*********************************************************************
 *          FreeRTOS运行时统计配置
 ********************************************************************/
/* 不使用运行时统计 */
#define configGENERATE_RUN_TIME_STATS           0

/* 不使用跟踪功能 */
#define configUSE_TRACE_FACILITY                0

#define configUSE_STATS_FORMATTING_FUNCTIONS    1

/*********************************************************************
 *          FreeRTOS协程配置
 ********************************************************************/
/* 不使用协程 */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         (2)

/*********************************************************************
 *          FreeRTOS软件定时器配置
 ********************************************************************/
/* 不使用软件定时器 */
#define configUSE_TIMERS                        0
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES-1)
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE*2)

/*********************************************************************
 *          FreeRTOS可选函数配置
 ********************************************************************/
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          0

/*********************************************************************
 *          FreeRTOS中断优先级配置
 ********************************************************************/
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS                     __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS                     4
#endif

/* 中断最低优先级 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15

/* 系统可管理的最高中断优先级 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/*********************************************************************
 *          FreeRTOS中断服务函数映射
 ********************************************************************/
#define xPortPendSVHandler      PendSV_Handler
#define vPortSVCHandler         SVC_Handler

#endif /* FREERTOS_CONFIG_H */
