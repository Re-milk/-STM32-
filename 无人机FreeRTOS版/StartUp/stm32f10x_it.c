/**
  ******************************************************************************
  * @file    stm32f10x_it.c
  * @brief   中断服务函数 - FreeRTOS版本
  *          注意: SVC_Handler和PendSV_Handler由FreeRTOS port.c处理
  *                SysTick_Handler需要调用xPortSysTickHandler()
  ******************************************************************************
  */

#include "stm32f10x_it.h"
#include "ALL_DATA.h"
#include "stdio.h"
#include "usart3.h"
#include "delay.h"

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

/*
 * 注意: SVC_Handler 和 PendSV_Handler 已在FreeRTOSConfig.h中
 * 通过宏定义映射到FreeRTOS的 vPortSVCHandler 和 xPortPendSVHandler
 * 由FreeRTOS的port.c文件提供实现，不要在这里重复定义
 */

void DebugMon_Handler(void)
{
}

/**
  * @brief  SysTick中断处理函数
  * @note   FreeRTOS需要在此调用xPortSysTickHandler()
  *         同时保留原有的应用层tick处理
  */
void SysTick_Handler(void)
{
    /* FreeRTOS tick处理 - 必须调用 */
    if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }

    /* 应用层tick处理 */
    SysTick_IRQ();
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
