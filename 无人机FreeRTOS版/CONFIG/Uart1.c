//========================================================================
//	USART1 驱动 - MAVLink 遥测口 (PA9 TX / PA10 RX)
//	接收: RXNE 中断 -> RX 环形缓冲 -> 任务里解析
//	发送: 任务写 TX 环形缓冲 -> TXE 中断逐字节发出（不阻塞任务）
//========================================================================

#include "Uart1.h"
#include "ALL_DEFINE.h"

#define UART1_RX_SIZE   128u
#define UART1_TX_SIZE   256u
#define UART1_RX_MASK   (UART1_RX_SIZE - 1u)
#define UART1_TX_MASK   (UART1_TX_SIZE - 1u)

/* 每个索引都只有一个写者，构成单生产者/单消费者环形缓冲，无需加锁 */
static volatile uint8_t  uart1_rx_buf[UART1_RX_SIZE];
static volatile uint16_t uart1_rx_head = 0;   /* 中断写 */
static volatile uint16_t uart1_rx_tail = 0;   /* 任务写 */

static volatile uint8_t  uart1_tx_buf[UART1_TX_SIZE];
static volatile uint16_t uart1_tx_head = 0;   /* 任务写 */
static volatile uint16_t uart1_tx_tail = 0;   /* 中断写 */

static volatile uint16_t uart1_rx_ovf = 0;
static volatile uint16_t uart1_tx_ovf = 0;

static uint8_t uart1_inited = 0;

/************************************************************************
 * @brief  USART1 初始化（8N1，无流控，使能 RXNE 中断）
 * @note   可重复调用：首次调用才真正初始化，后续直接返回，
 *         避免把已在发送中的数据冲掉（main 里会先初始化一次）
 ************************************************************************/
void ANO_Uart1_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	if(uart1_inited)
		return;
	uart1_inited = 1;

	uart1_rx_head = uart1_rx_tail = 0;
	uart1_tx_head = uart1_tx_tail = 0;
	uart1_rx_ovf  = 0;
	uart1_tx_ovf  = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_UART1, ENABLE);

	/* PA9 = TX */
	GPIO_InitStructure.GPIO_Pin   = UART1_Pin_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIO_UART1, &GPIO_InitStructure);

	/* PA10 = RX */
	GPIO_InitStructure.GPIO_Pin  = UART1_Pin_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIO_UART1, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate            = br_num;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

	/* 默认只开接收中断，TXE 由发送函数按需打开 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART1_P;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = NVIC_UART1_S;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/* 未使用，保留以兼容旧声明 */
void ANO_Uart1_DeInit(void)
{
}

/************************************************************************
 * @brief  USART1 中断：接收入队 + 发送排空
 * @note   本 ISR 不调用任何 FreeRTOS API
 ************************************************************************/
void USART1_IRQHandler(void)
{
	uint16_t sr = USART1->SR;

	/* ---------------- 接收 ---------------- */
	/* ORE 和 RXNE 都必须读 DR 才能清除，所以先取标志再读数据 */
	if(sr & (USART_FLAG_RXNE | USART_FLAG_ORE))
	{
		uint8_t data = (uint8_t)USART1->DR;

		if(sr & USART_FLAG_RXNE)
		{
			uint16_t next = (uint16_t)((uart1_rx_head + 1u) & UART1_RX_MASK);
			if(next != uart1_rx_tail)
			{
				uart1_rx_buf[uart1_rx_head] = data;
				uart1_rx_head = next;
			}
			else
			{
				uart1_rx_ovf++;          /* 缓冲满，丢弃最新字节 */
			}
		}
	}

	/* ---------------- 发送 ---------------- */
	if((sr & USART_FLAG_TXE) && (USART1->CR1 & USART_CR1_TXEIE))
	{
		if(uart1_tx_tail != uart1_tx_head)
		{
			USART1->DR = uart1_tx_buf[uart1_tx_tail];
			uart1_tx_tail = (uint16_t)((uart1_tx_tail + 1u) & UART1_TX_MASK);
		}
		else
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}
	}
}

/************************************************************************
 * @brief  从接收缓冲取 1 字节
 * @retval 1 = 取到；0 = 空
 ************************************************************************/
uint8_t UART1_RxPop(uint8_t *byte)
{
	uint16_t head = uart1_rx_head;          /* volatile 快照 */

	if(uart1_rx_tail == head)
		return 0;

	*byte = uart1_rx_buf[uart1_rx_tail];
	uart1_rx_tail = (uint16_t)((uart1_rx_tail + 1u) & UART1_RX_MASK);
	return 1;
}

uint16_t UART1_RxUsed(void)
{
	return (uint16_t)((uart1_rx_head - uart1_rx_tail) & UART1_RX_MASK);
}

void UART1_RxFlush(void)
{
	uart1_rx_tail = uart1_rx_head;
}

/************************************************************************
 * @brief  发送缓冲剩余空间
 ************************************************************************/
uint16_t UART1_TxFree(void)
{
	uint16_t head = uart1_tx_head;
	uint16_t tail = uart1_tx_tail;          /* 快照可能偏旧 => 只低估空闲量，安全 */
	uint16_t used = (uint16_t)((head - tail) & UART1_TX_MASK);

	return (uint16_t)(UART1_TX_SIZE - 1u - used);
}

/************************************************************************
 * @brief  把数据塞进发送缓冲后立即返回（真正发送由 TXE 中断完成）
 * @retval 1 = 已入队；0 = 空间不足，整帧丢弃
 ************************************************************************/
uint8_t UART1_WriteBuf(const uint8_t *data, uint16_t len)
{
	uint16_t i;
	uint32_t primask;

	if(len == 0u)
		return 1;

	if(UART1_TxFree() < len)
	{
		uart1_tx_ovf++;
		return 0;
	}

	for(i = 0; i < len; i++)
	{
		uart1_tx_buf[uart1_tx_head] = data[i];
		uart1_tx_head = (uint16_t)((uart1_tx_head + 1u) & UART1_TX_MASK);
	}

	/* 与 ISR 中"缓冲空就关 TXE"的操作互斥，避免最后一个字节卡在缓冲里 */
	primask = __get_PRIMASK();
	__disable_irq();
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	__set_PRIMASK(primask);

	return 1;
}

/************************ 兼容旧接口 ************************/
void ANO_Uart1_Put_Char(unsigned char DataToSend)
{
	(void)UART1_WriteBuf(&DataToSend, 1u);
}

void ANO_Uart1_Put_String(unsigned char *Str)
{
	while(*Str)
	{
		if(*Str == '\r')       (void)UART1_WriteBuf((const uint8_t *)"\r", 1u);
		else if(*Str == '\n')  (void)UART1_WriteBuf((const uint8_t *)"\n", 1u);
		else                   (void)UART1_WriteBuf((const uint8_t *)Str, 1u);
		Str++;
	}
}

void ANO_Uart1_Put_Buf(unsigned char *DataToSend, u8 data_num)
{
	(void)UART1_WriteBuf((const uint8_t *)DataToSend, (uint16_t)data_num);
}

/************************ 诊断 ************************/
uint16_t UART1_RxOverflowCount(void) { return uart1_rx_ovf; }
uint16_t UART1_TxOverflowCount(void) { return uart1_tx_ovf; }
