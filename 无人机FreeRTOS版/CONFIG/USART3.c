//========================================================================
//	USART3驱动
//========================================================================

#include "stm32f10x.h"                  // Device header
#include "usart3.h"
#include "misc.h"
#include "stdio.h"
#include "ALL_DEFINE.h"
#include "ELRS.h"

void USART3_Config(u32 br_num) 
{
	NVIC_InitTypeDef NVIC_InitStruct;
	GPIO_InitTypeDef GPIO_STRUCTURE_INIT;
	USART_InitTypeDef USART_INIT_STRUCTURE;
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	//��ʼ������
	GPIO_STRUCTURE_INIT.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_STRUCTURE_INIT.GPIO_Pin=GPIO_Pin_10;
	GPIO_STRUCTURE_INIT.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_STRUCTURE_INIT);    
	GPIO_STRUCTURE_INIT.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_STRUCTURE_INIT.GPIO_Pin=GPIO_Pin_11;
	GPIO_STRUCTURE_INIT.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_STRUCTURE_INIT);
	
	//����USART����
	USART_INIT_STRUCTURE.USART_BaudRate=br_num;
	USART_INIT_STRUCTURE.USART_HardwareFlowControl=DISABLE;
	USART_INIT_STRUCTURE.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_INIT_STRUCTURE.USART_Parity=USART_Parity_No;//�޼���֡
	USART_INIT_STRUCTURE.USART_StopBits=USART_StopBits_1;//����ֹͣ�ֽ�Ϊ1�ֽ�
	USART_INIT_STRUCTURE.USART_WordLength=USART_WordLength_8b;//����Ϊ��λ����֡
	USART_Init(USART3,&USART_INIT_STRUCTURE);//����usart
    
	//����USART3���߿����ж�
  USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_Init(&NVIC_InitStruct);    
	USART_Cmd(USART3,ENABLE);
}


//����3���ս���Elrs_CRSF���ַ�ʽ
//��ȡ��ʽ����DMA+���ڿ����ж�
//***************************************************************************************************
//uint8_t rc_tmp;
//void USART3_IRQHandler(void)
//{ 
//	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
//	{
//		rc_tmp=USART3->SR;
//		rc_tmp=USART3->DR;//�����������IDLE��־λ
//		DMA_Cmd(DMA1_Channel3,DISABLE);
//		DMA_SetCurrDataCounter(DMA1_Channel3,SIZEBUFF); //���賤���뵱�����ý��������±�Ϊ0
//		DMA_Cmd(DMA1_Channel3,ENABLE);
//		receiveCRSF(DataA) ;                                //DMA+�����жϽ��ղ�����Elrs_CRSF���ݰ��� rcChannels[i]  16��ͨ��
//		USART_ClearITPendingBit(USART3, USART_IT_IDLE); // ��������ж�
//	}
//}

//***************************************************************************************************


void USART3_SendByte(const int8_t *Data,uint8_t len)
{ 
	uint8_t i;
	
	for(i=0;i<len;i++)
	{
		while (!(USART3->SR & USART_FLAG_TXE));	 // while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		USART_SendData(USART3,*(Data+i));	 
	}
}

void USART3_Send1(const int8_t Data)  
{ 

		while (!(USART3->SR & USART_FLAG_TXE));	 // while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		USART_SendData(USART3,(Data));	 
	
}


int8_t CheckSend[7]={0xAA,0XAA,0xEF,2,0,0,0};

void USART1_setBaudRate(uint32_t baudRate)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate =  baudRate;
	USART_Init(USART1, &USART_InitStructure);
}

//int fputc(int ch, FILE *f)
//{
//	/*  */
//	USART_SendData(USART3, (unsigned char) ch);
//	while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET);	
//	return (ch);
//}


//////////////////////////////////////////////
////����3�жϳ�����  ANO_DT.C ��������(����λ��ͨ�ź�����
//////////////////////////////////////////////
//u8 TxBuffer3[256];
//u8 TxCounter3=0;
//u8 count3=0;
//void USART3_IRQHandler(void) //����3�жϽ��ճ���
//{ 	
//	if((USART3->SR & (1<<7))&&(USART3->CR1 & USART_CR1_TXEIE))
//	{
//		USART3->DR = TxBuffer3[TxCounter3++]; //дDR����жϱ�־ 
//		if(TxCounter3 == count3)
//		{
//			USART3->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�ж�
//		}
//	}
//	//�����ж�
//	if(USART3->SR & (1<<5))//if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)    
//	{
//		u8 com_data = USART3->DR;
// 	  ANO_DT_Data_Receive_Prepare(com_data);//��λ�����ݽ���
//	}
//
//}







