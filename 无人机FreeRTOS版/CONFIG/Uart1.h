#ifndef __UART_H__
#define __UART_H__

#include "stdio.h"
#include "stm32f10x.h"

/*
 * USART1 (PA9 = TX / PA10 = RX) 是 MAVLink 遥测口。
 * 收发都走环形缓冲 + 中断，任务侧调用不会阻塞。
 *
 * 原匿名上位机（ANO_DT）在此口上的用法已废弃；
 * ANO 协议仍保留在 NRF24L01 那条 2.4G 链路上，不受影响。
 */

void ANO_Uart1_Init(u32 br_num);          /* 兼容原函数名，br_num 建议 57600 */
void ANO_Uart1_DeInit(void);

/* ---------------- 接收：中断写入，任务读出 ---------------- */
uint8_t  UART1_RxPop(uint8_t *byte);      /* 返回 1 表示取到 1 字节 */
uint16_t UART1_RxUsed(void);              /* 缓冲中待读字节数 */
void     UART1_RxFlush(void);

/* ---------------- 发送：任务写入，中断排空 ---------------- */
uint16_t UART1_TxFree(void);              /* 还可写入多少字节 */
uint8_t  UART1_WriteBuf(const uint8_t *data, uint16_t len);  /* 1=已入队 0=空间不足 */

/* ---------------- 诊断 ---------------- */
uint16_t UART1_RxOverflowCount(void);
uint16_t UART1_TxOverflowCount(void);

/* 兼容旧代码的接口（内部转成 UART1_WriteBuf，非阻塞） */
void ANO_Uart1_Put_Char(unsigned char DataToSend);
void ANO_Uart1_Put_String(unsigned char *Str);
void ANO_Uart1_Put_Buf(unsigned char *DataToSend, u8 data_num);

extern int fputc(int ch, FILE *f);

#endif
