/**
  ******************************************************************************
  * @file    board.c
  * @brief   板级驱动 - FreeRTOS版本
  *          移除SysTick_IRQ()，SysTick由FreeRTOS管理
  ******************************************************************************
  */

#include "board.h"

static volatile uint32_t usTicks = 0;

uint32_t SysTick_count = 0;

void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

/**
  * @brief  简单延时函数
  */
void Delay(vu32 nCount)
{
    for (; nCount != 0; nCount--);
}

/*
 * 注意: SysTick_IRQ() 在FreeRTOS版本中不再需要
 * SysTick_Handler() 在 stm32f10x_it.c 中由FreeRTOS接管
 * 原始项目的 Loop_check() 调度逻辑已替换为FreeRTOS多任务
 */
