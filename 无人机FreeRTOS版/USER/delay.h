#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void cycleCounterInit(void);
void SysTick_IRQ(void);
uint32_t GetSysTime_us(void);
void delay_ms(uint16_t nms);
void delay_us(unsigned int i);
float micros(void);

#endif
