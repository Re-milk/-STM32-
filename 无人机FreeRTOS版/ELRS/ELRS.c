//========================================================================
//	ELRS驱动
//========================================================================

#include "ELRS.h"
#include "ALL_DATA.h"
#include "ALL_DEFINE.h"

/* 全局变量定义 */
uint8_t SIZEBUFF = 26;
uint8_t DataA[26];
uint16_t rcChannels[CRSF_MAX_CHANNEL];

void Dma_init(void)
{
		DMA_InitTypeDef DMA_InitTypeStruct;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);	
	DMA_DeInit(DMA1_Channel3);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ    
	DMA_InitTypeStruct.DMA_PeripheralBaseAddr = (uint32_t) &USART3->DR; //�������ַ
	DMA_InitTypeStruct.DMA_PeripheralDataSize =  DMA_PeripheralDataSize_Byte;  //���贫���С
	DMA_InitTypeStruct.DMA_PeripheralInc =  DMA_PeripheralInc_Disable; //��������ģʽ
	DMA_InitTypeStruct.DMA_MemoryBaseAddr = (uint32_t)&DataA; //�洢������ַ
	DMA_InitTypeStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�������С `
	DMA_InitTypeStruct.DMA_MemoryInc =  DMA_MemoryInc_Enable; //�洢������ģʽ
	DMA_InitTypeStruct.DMA_DIR = DMA_DIR_PeripheralSRC; //������Ϊ��Դ
	DMA_InitTypeStruct.DMA_BufferSize = SIZEBUFF;    //��������С
	DMA_InitTypeStruct.DMA_Mode =  DMA_Mode_Circular; //DMAģʽ  ѭ��
	DMA_InitTypeStruct.DMA_M2M = DMA_M2M_Disable;  //�洢�����洢��ģʽ��
	DMA_InitTypeStruct.DMA_Priority =  DMA_Priority_Medium;  //ͨ�����ȼ�
	DMA_Init(DMA1_Channel3,&DMA_InitTypeStruct);
	
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);    
	DMA_Cmd(DMA1_Channel3,ENABLE);    //ʹ��usart1���������շ�
}

uint8_t _lut[256];
uint8_t calcCRC(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc = _lut[crc ^ *data++];
    }
    return crc;
}

// ������Ч��Elrs_CRSF���ݰ��� rcChannels[i]  16��ͨ��
void processCRSFPacket(uint8_t *packet) 
{
    // ������ݰ��������Ƿ�Ϊң��ͨ�����ݰ�
    if (packet[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER && packet[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)  //#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8      // �ɿص�ַ
		{                                                                                                     // #define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16  // ң��ͨ�����ݰ����� 
        uint32_t bitBuffer = 0;                                                                       
        uint8_t bitsInBuffer = 0;
        int byteIndex = 3; // ���ݲ��ִӵ�3���ֽڿ�ʼ
        int i; 
        // ����16��ͨ����11λ����
        for (i = 0; i < CRSF_MAX_CHANNEL; i++) 
				{
            while (bitsInBuffer < 11) 
						{
                bitBuffer |= ((uint32_t)packet[byteIndex++] << bitsInBuffer);  // ������������ȡ�ֽڲ���λ
                bitsInBuffer += 8;
            }
            rcChannels[i] = bitBuffer & 0x07FF; // ȡ��11λ��Ϊͨ��ֵ
            bitBuffer >>= 11;
            bitsInBuffer -= 11;
        }			
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////����Elrs_CSRF������ʽһ��DMA+���ڿ����ж�
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void receiveCRSF(uint8_t *packet)                                                //DMA+���ڿ����жϽ��ղ�����Elrs_CRSF���ݰ�       
{
				//uint8_t calculatedCRC;
        // //���CRCУ��
		  	//calculatedCRC = calcCRC(&DataA[2], CRSF_FRAME_SIZE - 1); // �ӵ�2�ֽڵ�������2�ֽڽ���CRCУ��
		  	//if (calculatedCRC == DataA[CRSF_FRAME_SIZE + 1]) 
		  	//{
		  		processCRSFPacket(packet);	
		  	//}	
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void CRSF_Connect(void)
{  
		 Remote.thr = rcChannels[2]*0.612f+893.51f ;	 
	   Remote.yaw = rcChannels[3]*0.612f+893.51f  ;
	   Remote.pitch = rcChannels[0]*0.612f+893.51f +roll66 ;      //���ʱ�������ҷ���   -������������ +������������ ;
	   Remote.roll = rcChannels[1]*0.612f+893.51f  +pitch66;      //���ʱ����ǰ����   -����������� +������ǰ���� ;   
	// Remote.AUX1 = rcChannels[4] ;
	   Remote.AUX2 = rcChannels[6] ;
	// Remote.AUX3 = rcChannels[5] ;
	// Remote.AUX4 = rcChannels[7] ;
	// Remote.AUX5 = rcChannels[8] ;
	// Remote.AUX6 = rcChannels[9] ;	
}






