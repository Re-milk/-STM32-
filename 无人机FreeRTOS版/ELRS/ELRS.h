#ifndef __ELRS_H
#define __ELRS_H
#include "stm32f10x.h"                  // Device header


// 定义CRSF协议相关常量
#define CRSF_MAX_CHANNEL 16          // CRSF协议支持的最大通道数
#define CRSF_FRAME_SIZE 24           // CRSF数据帧总大小（包含CRC）
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8    // 飞控地址
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16 // 遥控通道数据包类型

extern uint8_t	SIZEBUFF;
extern uint8_t DataA[26];
extern uint16_t rcChannels[CRSF_MAX_CHANNEL];


void Dma_init(void);
void processCRSFPacket(uint8_t *packet) ;              // 处理有效的Elrs_CRSF数据包给 rcChannels[i]  16个通道
void receiveCRSF(uint8_t *packet);                     //DMA+串口空闲中断接收并解析Elrs_CRSF数据包

void CRSF_Connect(void);

#endif

