/**
  ******************************************************************************
  * @file    stm32f10x_it.c
  * @brief   中断服务函数 - FreeRTOS版本
  *          SysTick由FreeRTOS接管，不再调用原Loop_check()
  ******************************************************************************
  */

#include "stm32f10x_it.h"
#include "usb_istr.h"
#include "ANO_Drv_Uart.h"

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"

/**
  * @brief  串口1中断
  */
void USART1_IRQHandler(void)
{
    Uart1_IRQ();
}

/**
  * @brief  SysTick中断 - 由FreeRTOS接管
  *         原始项目中SysTick调用Loop_check()进行调度
  *         FreeRTOS版本中SysTick由FreeRTOS内核管理
  */
extern void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
    #if (INCLUDE_xTaskGetSchedulerState == 1)
      if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
      {
    #endif
        xPortSysTickHandler();
    #if (INCLUDE_xTaskGetSchedulerState == 1)
      }
    #endif
}

/**
  * @brief  USB高优先级中断
  */
void USB_HP_CAN1_TX_IRQHandler(void)
{
}

/**
  * @brief  USB低优先级中断
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    USB_Istr();
}
