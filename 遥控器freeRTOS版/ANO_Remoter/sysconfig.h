#ifndef _SYSCONFIG_H_
#define _SYSCONFIG_H_
#include "stm32f10x.h"
#include "board.h"
#include "ANO_Stick.h"
#include "ANO_DT.h"
#include "show.h"
#include "oled.h"
#include "key.h"
#include "ANO_Param.h"
#include "ANO_Drv_LED.h"

/* FreeRTOS版本: 移除了ANO_Scheduler.h的包含
 * 调度逻辑已由FreeRTOS多任务替代
 * 原始ANO_Scheduler.c中的任务函数直接在main.c的任务中调用
 */

#endif
