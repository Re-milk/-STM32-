//========================================================================
//	延时函数 - FreeRTOS版本
//	使用FreeRTOS的系统节拍实现延时
//========================================================================

#include "stm32f10x.h"
#include "misc.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile uint32_t usTicks = 0;

volatile uint32_t SysTick_count22 = 0;
u8 sys_init_ok = 1;

void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

void SysTick_IRQ(void)
{
    /* FreeRTOS的tick由xPortSysTickHandler处理 */
    /* 这里只处理应用层的计数 */
    SysTick_count22++;
}

uint32_t GetSysTime_us(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = SysTick_count22;
        cycle_cnt = SysTick->VAL;
    } while (ms != SysTick_count22);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

/* 毫秒级延时 - 使用FreeRTOS延时 */
void delay_ms(uint16_t nms)
{
    if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        /* 调度器已启动，使用FreeRTOS延时 */
        vTaskDelay(nms);
    }
    else
    {
        /* 调度器未启动，使用忙等待延时 */
        uint32_t t0 = GetSysTime_us();
        while(GetSysTime_us() - t0 < nms * 1000);
    }
    SysTick_count22++;
}

/* 微秒级延时 - 忙等待 */
void delay_us(unsigned int i)
{
    char x = 0;
    while(i--)
    {
        for(x = 1; x > 0; x--);
    }
}

/* 获取系统当前时间 单位ms */
float micros(void)
{
    return SysTick_count22 + (SysTick->LOAD - SysTick->VAL) / (72000.0f / 8.0f);
}

/********************************END OF FILE*****************************************/
