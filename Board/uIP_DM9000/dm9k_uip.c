
/****************************************************************************
* Copyright (C), 2009-2010, www.armfly.com  安富莱电子
*
* 【本驱动在安富莱STM32F103ZE-EK开发板上调试通过             】
* 【QQ: 1295744630, 旺旺：armfly, Email: armfly@qq.com       】
*
* 文件名: dm9k_uip.c
* 内容简述: Davicom DM9000A uP NIC fast Ethernet driver for uIP.
*
* 文件历史:
* 版本号  日期       作者    说明
* v0.1    2010-01-18 armfly  创建该文件
*
*/
//#define DM9000A_FLOW_CONTROL
//#define DM9000A_UPTO_100M
//#define Fifo_Point_Check
//#define Point_Error_Reset
#define Fix_Note_Address
#include <stdio.h>
#include "stm32f10x.h"
#include  "uip.h"
#include  "dm9k_uip.h"
#include "ucos_ii.h"
#include "printf_level.h"

/* DM9000A 接收函数设置宏 */
#define Rx_Int_enable
#define Max_Int_Count			1
//#define Max_Ethernet_Lenth		1536
#define Broadcast_Jump
#define Max_Broadcast_Lenth		500

#define DM9000_PKT_RDY		0x01	/* Packet ready to receive */
#define DM9000_PKT_MAX		1536	/* Received packet max size */


/* DM9000A 传送函数设置宏 */
#define Max_Send_Pack			2

/* 系统全域的设置宏
#undef    uint8_t
#define   uint8_t				unsigned char
#undef    uint16_t
#define   uint16_t				unsigned short
#undef    UINT32
#define   UINT32				unsigned long
*/

#define   NET_BASE_ADDR		0x6C100000
#define   NET_REG_ADDR			(*((volatile uint16_t *) NET_BASE_ADDR))
#define   NET_REG_DATA			(*((volatile uint16_t *) (NET_BASE_ADDR + 8)))

//#define FifoPointCheck

/*=============================================================================
  系统全域的变量
  =============================================================================*/
#define ETH_ADDR_LEN			6

OS_EVENT *dm9000_rx_sem=NULL;
OS_EVENT *dm9000_rx_mutex=NULL;
OS_EVENT *dm9000_tx_sem=NULL;
static unsigned char DEF_MAC_ADDR[ETH_ADDR_LEN] =
	{0x00, 0x60, 0x6e, 0x90, 0x00, 0xae};
uint32_t  TxPktCnt = 0;
uint32_t  RxPktCnt = 0;
uint16_t g_RxLen=0;
uint16_t queue_pkt_len=0;
uint16_t g_TxComplete=1;
uint16_t g_RxFlag=0;
uint16_t g_MaxEvent=0;
uint16_t g_CurEvent=0;

uint8_t s_FSMC_Init_Ok = 0;	/* 用于指示FSMC是否初始化 */

#define printk(...)	printf(__VA_ARGS__)
//#define printk printf

void dm9k_debug_test(void);
void dm9k_udelay(uint16_t time);

static void DM9K_CtrlLinesConfig(void);
static void DM9K_FSMCConfig(void);
static void DM9K_IntModeInit(void);


/*******************************************************************************
*	函数名: dm9k_udelay
*	参  数: time ： 延迟时间，不精确，us级别
*	返  回: 无
*	功  能: 延迟函数
*/
void dm9k_udelay(uint16_t time)
{
    uint16_t i,k;

	for (i = 0; i < time; i++)
	{
		for (k = 0; k < 80; k++);
	}
	while(time--);
}

/*******************************************************************************
*	函数名: ior
*	参  数: reg ：寄存器地址
*	返  回: 无
*	功  能: 读出寄存器的值
*/
uint8_t ior(uint8_t reg)
{
	NET_REG_ADDR = reg;
	return (NET_REG_DATA);
}

/*******************************************************************************
*	函数名: iow
*	参  数: reg ：寄存器地址
*			writedata : 写入的数据
*	返  回: 无
*	功  能: 写DM9000AE寄存器的值
*/
void iow(uint8_t reg, uint8_t writedata)
{
	NET_REG_ADDR = reg;
	NET_REG_DATA = writedata;
}

/*******************************************************************************
*	函数名: dm9k_hash_table
*	参  数: 无
*	返  回: 无
*	功  能: 设置 DM9000A MAC 、 广播 、 多播 寄存器
*/
void dm9k_hash_table(void)
{
	uint8_t i;

	/* 将MAC地址告诉uip */
	for (i = 0; i < 6; i++)
	{
		uip_ethaddr.addr[i] = DEF_MAC_ADDR[i];
	}

	/* 设置 网卡 MAC 位置，来自於 MyHardware */
	for(i = 0; i < 6; i++)
		iow(DM9000_REG_PAR + i, DEF_MAC_ADDR[i]);

	for(i = 0; i < 8; i++) 								/* 清除 网卡多播设置 */
		iow(DM9000_REG_MAR + i, 0x00);
	iow(DM9000_REG_MAR + 7, 0x80);  					/* 速设置 广播包 设置 */
}

/*******************************************************************************
*	函数名: dm9k_reset
*	参  数: 无
*	返  回: 无
*	功  能: 对DM9000AE进行软件复位
*/
void dm9k_reset(void)
{
	iow(DM9000_REG_NCR, DM9000_REG_RESET);			/* 对 DM9000A 进行软件重置 */
	dm9k_udelay(10);								/* delay 10us */
	iow(DM9000_REG_NCR, DM9000_REG_RESET);			/* 对 DM9000A 进行软件重置 */
	dm9k_udelay(10);								/* delay 10us */

	/* 基本记存器相关设置 */
	iow(DM9000_REG_IMR, DM9000_IMR_OFF); 			/* 开启内存自环模式 */
	iow(DM9000_REG_TCR2, DM9000_TCR2_SET);			/* 设置 LED 显示模式1:全双工亮，半双工灭 */

	/* 清除多余资讯 */
	iow(DM9000_REG_NSR, 0x2c);
	iow(DM9000_REG_TCR, 0x00);
	iow(DM9000_REG_ISR, 0x3f);

#ifdef DM9000A_FLOW_CONTROL
	iow(DM9000_REG_BPTR, DM9000_BPTR_SET);			/* 半双工流控设置 */
	iow(DM9000_REG_FCTR, DM9000_FCTR_SET);			/* 全双工流控设置 */
	iow(DM9000_REG_FCR, DM9000_FCR_SET);			/* 开启流控设置 */
#endif

#ifdef DM9000A_UPTO_100M
	/* DM9000A无此寄存器 */
	iow(DM9000_REG_OTCR, DM9000_OTCR_SET);			/* 工作频率到 100Mhz 设置 */
#endif

#ifdef  Rx_Int_enable
	iow(DM9000_REG_IMR, DM9000_IMR_SET);			/* 开启 中断模式 */
#else
	iow(DM9000_REG_IMR, DM9000_IMR_OFF);			/* 关闭 中断模式 */
#endif

	iow(DM9000_REG_RCR, DM9000_RCR_SET);			/* 开启 接收工能 */

	TxPktCnt = 0;
	RxPktCnt = 0;
}

/*******************************************************************************
*	函数名: dm9k_phy_write
*	参  数: phy_reg ： 寄存器地址
*			writedata ： 写入的数据
*	返  回: 无
*	功  能: 写DM9000A PHY 寄存器
*/
void dm9k_phy_write(uint8_t phy_reg, uint16_t writedata)
{
	/* 设置写入 PHY 寄存器的位置 */
	iow(DM9000_REG_EPAR, phy_reg | DM9000_PHY);

	/* 设置写入 PHY 寄存器的值 */
	iow(DM9000_REG_EPDRH, ( writedata >> 8 ) & 0xff);
	iow(DM9000_REG_EPDRL, writedata & 0xff);

	iow(DM9000_REG_EPCR, 0x0a); 						/* 将资料写入 PHY 寄存器 */
	while(ior(DM9000_REG_EPCR) & 0x01);					/* 查寻是否执行结束 */
	iow(DM9000_REG_EPCR, 0x08); 						/* 清除写入命令 */
}

/*******************************************************************************
*	函数名: dm9k_initnic
*	参  数: 无
*	返  回: 无
*	功  能: 初始化DM9000AE
*/
void dm9k_initnic(void)
{
	iow(DM9000_REG_NCR, DM9000_REG_RESET);			/* 对 DM9000A 进行软件重置 */
	dm9k_udelay(10);								/* delay 10us */

	dm9k_hash_table();								/* 设置 DM9000A MAC 及 多播*/

	dm9k_reset();									/* 进行 DM9000A 软件设置 */

	iow(DM9000_REG_GPR, DM9000_PHY_OFF);			/* 关闭 PHY ，进行 PHY 设置*/
	dm9k_phy_write(0x00, 0x8000);					/* 重置 PHY 的寄存器 */
#ifdef DM9000A_FLOW_CONTROL
	dm9k_phy_write(0x04, 0x01e1 | 0x0400);			/* 设置 自适应模式相容表 */
#else
	dm9k_phy_write(0x04, 0x01e1);					/* 设置 自适应模式相容表 */
#endif
	//dm9k_phy_write(0x00, 0x1000);					/* 设置 基本连接模式 */
	/* 连接模式设置
	  0x0000 : 固定10M半双工
	  0x0100 : 固定10M全双工
	  0x2000 : 固定100M半双工
	  0x2100 : 固定100M全双工
	  0x1000 : 自适应模式
	*/
	dm9k_phy_write(0x00, 0x1000);				/* 设置 基本连接模式 */

	iow(DM9000_REG_GPR, DM9000_PHY_ON);				/* 结束 PHY 设置, 开启 PHY */

	//dm9k_debug_test();
}


static void DM9K_IntModeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable Button GPIO clock */
	RCC_APB2PeriphClockCmd(DM9000_IRQ_PORT_CLK | RCC_APB2Periph_AFIO, ENABLE);
	
	/* Configure Button pin as input floating */
	//GPIO_InitStructure.GPIO_Pin = MRFI_GDO0_PIN | MRFI_GDO2_PIN;
	GPIO_InitStructure.GPIO_Pin = DM9000_IRQ_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DM9000_IRQ_PORT, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(DM9000_IRQ_PORT_SOURCE, DM9000_IRQ_PIN_SOURCE);
	//GPIO_EXTILineConfig(MRFI_GDO_PORT_SOURCE, MRFI_GDO2_PIN_SOURCE);
	
	/* Configure Button EXTI line */
	//EXTI_InitStructure.EXTI_Line = MRFI_GDO0_LINE | MRFI_GDO2_LINE;
	EXTI_InitStructure.EXTI_Line = DM9000_IRQ_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = DM9000_IRQ_TRIGGER;
	//EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/* Enable and set Button EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = DM9000_IRQ_CHANNEL;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
*	函数名: dm9k_receive_packet
*	参  数: _uip_buf : 接收缓冲区
*	返  回: > 0 表示接收的数据长度, 0表示没有数据
*	功  能: 读取一包数据
*/
uint16_t dm9k_Rx(uint8_t *_uip_buf)
{
//	uint16_t ReceiveLength;
	uint16_t *ReceiveData;
//	uint8_t  rx_int_count = 0;
	uint8_t  rx_checkbyte,GoodPacket;
	uint16_t rx_status, rx_length;
//	uint8_t  jump_packet;
	uint16_t i;
	uint16_t calc_len;
//	uint16_t calc_MRR;
	//uint8_t  save_reg;
	//iow(DM9000_REG_IMR , DM9000_IMR_OFF);				/* 关闭 DM9000A 中断 */
	//save_reg = NET_REG_ADDR;
	
//	ReceiveLength = 0;								/* 清除接收的长度 */
	ReceiveData = (uint16_t *)_uip_buf;
//	jump_packet = 0;								/* 清除跳包动作 */
	ior(DM9000_REG_MRCMDX);							/* 读取内存数据，地址不增加 */
	//NET_REG_ADDR = DM9000_REG_MRCMDX;
	rx_checkbyte = NET_REG_DATA;			/*  */
	if (rx_checkbyte > DM9000_PKT_RDY) {
		printk("status check failed: %d\n", rx_checkbyte);
		iow(DM9000_REG_RCR,0x00);	/* Stop Device */
		iow(DM9000_REG_ISR,IMR_PAR);/* Stop INT request */
		return 0;
	}
	if(rx_checkbyte != DM9000_PKT_RDY)
		return 0;
	/* 计算内存数据位置 */
	//calc_MRR = (((uint16_t)ior(DM9000_REG_MRRH)) << 8) + ior(DM9000_REG_MRRL);

	GoodPacket = TRUE;
	/* 读取封包相关资讯 及 长度 */
	NET_REG_ADDR = DM9000_REG_MRCMD;
	rx_status = NET_REG_DATA;
	rx_length = NET_REG_DATA;

	/* Packet Status check */
	if (rx_length < 0x40) {
		GoodPacket = FALSE;
		printk("Bad Packet received (runt)\n");
	}

	if (rx_length > DM9000_PKT_MAX) {
		printk("RST: RX Len:%x\n", rx_length);
	}

	if (rx_status & 0xbf00) {
		GoodPacket = FALSE;
		if (rx_status & 0x100) {
			printk("fifo error\n");
			//db->stats.rx_fifo_errors++;
		}
		if (rx_status & 0x200) {
			printk("crc error\n");
			//db->stats.rx_crc_errors++;
		}
		if (rx_status & 0x8000) {
			printk("length error\n");
			//db->stats.rx_length_errors++;
		}
	}
	calc_len = (rx_length + 1) >> 1;
	if (GoodPacket) {
		/* 开始将内存的资料搬到到系统中，每次移动一个 word */
		for(i = 0 ; i < calc_len ; i++)
			ReceiveData[i] = NET_REG_DATA;

	} else {
		/* need to dump the packet's data */
		for(i = 0 ; i < calc_len ; i++)
			rx_status = NET_REG_DATA;
		return 0;
	}
	do{
		ior(DM9000_REG_MRCMDX);							/* 读取内存数据，地址不增加 */
		//NET_REG_ADDR = DM9000_REG_MRCMDX;
		rx_checkbyte = NET_REG_DATA;			/*  */
		if(rx_checkbyte==0)break;
		NET_REG_ADDR = DM9000_REG_MRCMD;
		rx_status = NET_REG_DATA;
		calc_len = NET_REG_DATA;
		calc_len = (calc_len + 1) >> 1;
		for(i = 0 ; i < calc_len ; i++)
			rx_status = NET_REG_DATA;
	}while(1);	
	//NET_REG_ADDR = save_reg;
	//iow(DM9000_REG_IMR , DM9000_IMR_SET);				/* 开启 DM9000A 中断 */
	return rx_length-4;
}

/*******************************************************************************
*	函数名: dm9k_send_packet
*	参  数: p_char : 发送数据缓冲区
*			length : 数据长度
*	返  回: 无
*	功  能: 发送一包数据
*/
#if 0
void dm9k_Tx(uint8_t *p_char, uint16_t length)
{
	uint16_t SendLength = length;
	uint16_t *SendData = (uint16_t *) p_char;
	uint16_t i;
	uint16_t calc_len;
	__IO uint16_t calc_MWR;

	/* 检查 DM9000A 是否还在传送中！若是等待直到传送结束 */
#if 1
	if(TxPktCnt == Max_Send_Pack)
	{
		//while(ior(DM9000_REG_TCR) & DM9000_TCR_SET)
		while(0==(ior(DM9000_REG_ISR) & DM9000_TX_INTR))
		{
			//dm9k_udelay (5);
			OSTimeDly(1);
		}
		iow(DM9000_REG_ISR,DM9000_TX_INTR);
		if(TxPktCnt)TxPktCnt--;
	}

	TxPktCnt++;										/* 设置传送计数 */
#else
	while(g_TxComplete);
	g_TxComplete=0;
#endif
	
#ifdef FifoPointCheck
	/* 计算下一个传送的指针位 , 若接收长度为奇数，需加一对齐偶字节。*/
	/* 若是超过 0x0bff ，则需回归绕到 0x0000 起始位置 */
	calc_MWR = (ior(DM9000_REG_MWRH) << 8) + ior(DM9000_REG_MWRL);
	calc_MWR += SendLength;
	if(SendLength & 0x01) calc_MWR++;
	if(calc_MWR > 0x0bff) calc_MWR -= 0x0c00;
#endif

	iow(DM9000_REG_TXPLH, (SendLength >> 8) & 0xff);	/* 设置传送封包的长度 */
	iow(DM9000_REG_TXPLL, SendLength & 0xff);

	/* 开始将系统的资料搬到到内存中，每次移动一个 word */
	NET_REG_ADDR = DM9000_REG_MWCMD;
	calc_len = (SendLength + 1) >> 1;
	for(i = 0; i < calc_len; i++)
		NET_REG_DATA = SendData[i];

	iow(DM9000_REG_TCR, DM9000_TCR_SET);				/* 进行传送 */

#ifdef FifoPointCheck
	if(calc_MWR != ((ior(DM9000_REG_MWRH) << 8) + ior(DM9000_REG_MWRL)))
	{
#ifdef Point_Error_Reset
		/* 若是指针出错，等待此一封包送完 , 之後进行重置 */
		while(ior(DM9000_REG_TCR) & DM9000_TCR_SET) dm9k_udelay (5);
		dm9k_reset();
		return;
#endif
		/*若是指针出错，将指针移到下一个传送包的包头位置  */
		iow(DM9000_REG_MWRH, (calc_MWR >> 8) & 0xff);
		iow(DM9000_REG_MWRL, calc_MWR & 0xff);
	}
#endif
	return;
}
#else
void dm9k_Tx(uint8_t *p_char, uint16_t length)
{
	uint16_t calc_len,i;
	uint16_t *SendData = (uint16_t *) p_char;
	if(TxPktCnt>1)return;
	//iow(DM9000_REG_IMR,DM9000_IMR_OFF);

	NET_REG_ADDR=DM9000_REG_MWCMD;
	calc_len = (length + 1) >> 1;
	for(i = 0; i < calc_len; i++)
		NET_REG_DATA = SendData[i];
	if(TxPktCnt==0)
	{
		TxPktCnt++;
		iow(DM9000_REG_TXPLL,length & 0xff);
		iow(DM9000_REG_TXPLH,length >> 8);
		iow(DM9000_REG_TCR,DM9000_TCR_SET);
	}
	else
	{
		TxPktCnt++;
		queue_pkt_len = length;
	}

	//iow(DM9000_REG_IMR,DM9000_IMR_SET);
}
void dm9k_Tx_Done(void)
{
	uint8_t status=ior(DM9000_REG_NSR);
	if(status & (NSR_TX2END | NSR_TX1END))
	{
		TxPktCnt--;
		if(TxPktCnt>0)
		{
			iow(DM9000_REG_TXPLL,queue_pkt_len & 0xff);
			iow(DM9000_REG_TXPLH,queue_pkt_len >> 8);
			iow(DM9000_REG_TCR,DM9000_TCR_SET);
		}
	}
}

#endif
/*******************************************************************************
*	函数名: dm9k_interrupt
*	参  数: 无
*	返  回: 无
*	功  能: 中断处理函数 (webserver例程未使用中断)
*/
void  dm9k_interrupt(void)
{
	uint8_t  save_reg;
	uint8_t  isr_status;
	save_reg = NET_REG_ADDR;							/* 暂存所使用的位置 */
	iow(DM9000_REG_IMR , DM9000_IMR_OFF);				/* 关闭 DM9000A 中断 */

	isr_status = ior(DM9000_REG_ISR);					/* 取得中断产生值 */
	iow(DM9000_REG_ISR,isr_status);						/* 请中断标志 */
	
	//printf_0("dm9k:%02x\n",isr_status);

#if 0  /* armfly */

#else
	if(isr_status & DM9000_RX_INTR) 					/* 检查是否为接收中断 */
	{
		//ReceivePackCount++;
		//g_RxLen=dm9k_Rx(uip_buf);							/* 执行接收处理程序 */
		//iow(DM9000_REG_ISR, DM9000_RX_INTR);			/*清接受中断标志*/
		OSSemPost(dm9000_rx_sem);
		g_CurEvent=dm9000_rx_sem->OSEventCnt;
		if(g_MaxEvent<g_CurEvent)g_MaxEvent=g_CurEvent;
		//(g_CurEvent==0)iow(DM9000_REG_RCR, DM9000_RCR_SET);			/* 开启 接收工能 */
		//if(g_CurEvent==0)iow(DM9000_REG_RCR, 0x30);			/* 关闭 接收工能 */
		//OSMutexPost(dm9000_rx_mutex);
	}
	if(isr_status & DM9000_TX_INTR) 					/* 检查是否为发送中断 */
	{
		//iow(DM9000_REG_ISR, DM9000_TX_INTR);			/*清接受中断标志*/	
		dm9k_Tx_Done();
	}
#endif

	iow(DM9000_REG_IMR , DM9000_IMR_SET);				/* 开启 DM9000A 中断 */
	NET_REG_ADDR = save_reg;							/* 回复所使用的位置 */
}

/*******************************************************************************
*	函数名: dm9k_debug_test
*	参  数: 无
*	返  回: 无
*	功  能: 测试DM9000AE的函数,用于排错
*/
void dm9k_debug_test(void)
{
	uint32_t check_device;
	uint8_t  check_iomode;
	uint8_t  check_reg_fail = 0;
	uint8_t  check_fifo_fail = 0;
	uint16_t i;
	uint16_t j;

	iow(DM9000_REG_NCR, DM9000_REG_RESET);			/* 对 DM9000A 进行软件重置 */
	dm9k_udelay(10);								/* delay 10us */
	iow(DM9000_REG_NCR, DM9000_REG_RESET);			/* 对 DM9000A 进行软件重置 */
	dm9k_udelay(10);								/* delay 10us */

	check_device  = ior(DM9000_REG_VID_L);
	check_device |= ior(DM9000_REG_VID_H) << 8;
	check_device |= ior(DM9000_REG_PID_L) << 16;
	check_device |= ior(DM9000_REG_PID_H) << 24;

	if(check_device != 0x90000A46)
	{
		printk("DM9K_DEBUG ==> DEIVCE NOT FOUND, SYSTEM HOLD !!\n");
		while(1);
	}
	else
	{
		printk("DM9K_DEBUG ==> DEIVCE FOUND !!\n");
	}

	check_iomode = ior(DM9000_REG_ISR) >> 6;
	if(check_iomode != DM9000_WORD_MODE)
	{
		printk("DM9K_DEBUG ==> DEIVCE NOT WORD MODE, SYSTEM HOLD !!\n");
		while(1);
	}
	else
	{
		printk("DM9K_DEBUG ==> DEIVCE IS WORD MODE !!\n");
	}

	printk("DM9K_DEBUG ==> REGISTER R/W TEST !!\n");
	NET_REG_ADDR = DM9000_REG_MAR;
	for(i = 0; i < 0x0100; i++)
	{
		NET_REG_DATA = i;
		if(i != (NET_REG_DATA & 0xff))
		{
			printk("             > error W %02x , R %02x \n", i , NET_REG_DATA);
			check_reg_fail = 1;
		}
	}

	if(check_reg_fail)
	{
		printk("DM9K_DEBUG ==> REGISTER R/W FAIL, SYSTEM HOLD !!\n");
		while(1);
	}

	printk("DM9K_DEBUG ==> FIFO R/W TEST !!\n");
	printk("DM9K_DEBUG ==> FIFO WRITE START POINT 0x%02x%02x \n",
			ior(DM9000_REG_MWRH), ior(DM9000_REG_MWRL));

	NET_REG_ADDR = DM9000_REG_MWCMD;
	for(i = 0; i < 0x1000; i++)
		NET_REG_DATA = ((i & 0xff) * 0x0101);

	printk("DM9K_DEBUG ==> FIFO WRITE END POINT 0x%02x%02x \n",
			ior(DM9000_REG_MWRH), ior(DM9000_REG_MWRL));

	if((ior(DM9000_REG_MWRH) != 0x20) || (ior(DM9000_REG_MWRL) != 0x00))
	{
		printk("DM9K_DEBUG ==> FIFO WRITE FAIL, SYSTEM HOLD !!\n");
		while(1);
	}

	ior(DM9000_REG_MRCMDX);
	printk("DM9K_DEBUG ==> FIFO READ START POINT 0x%02x%02x \n",
			ior(DM9000_REG_MRRH), ior(DM9000_REG_MRRL));
	ior(DM9000_REG_MRCMDX);

	NET_REG_ADDR = DM9000_REG_MRCMD;
	for(i = 0; i < 0x1000; i++)
	{
		j = NET_REG_DATA;

		if(((i & 0xff) * 0x0101) != j)
		{
			//printk("             > error W %04x , R %04x \n",
			//		((i & 0xff) * 0x0101) , j);
			check_fifo_fail = 1;
		}
	}

	printk("DM9K_DEBUG ==> FIFO READ END POINT 0x%02x%02x \n",
			ior(DM9000_REG_MRRH), ior(DM9000_REG_MRRL));

	if((ior(DM9000_REG_MRRH) != 0x20) || (ior(DM9000_REG_MRRL) != 0x00))
	{
		printk("DM9K_DEBUG ==> FIFO WRITE FAIL, SYSTEM HOLD !!\n");
		while(1);
	}

		if(check_fifo_fail)
	{
		printk("DM9K_DEBUG ==> FIFO R/W DATA FAIL, SYSTEM HOLD !!\n");
		while(1);
	}

	printk("DM9K_DEBUG ==> PACKET SEND & INT TEST !! \n");
	iow(DM9000_REG_NCR, DM9000_REG_RESET);
	dm9k_udelay(10);
	iow(DM9000_REG_NCR, DM9000_REG_RESET);
	dm9k_udelay(10);

	iow(DM9000_REG_IMR, DM9000_IMR_OFF | DM9000_TX_INTR);

	iow(DM9000_REG_TXPLH, 0x01);
	iow(DM9000_REG_TXPLL, 0x00);

	do
	{
		iow(DM9000_REG_ISR, DM9000_TX_INTR);
		printk("DM9K_DEBUG ==> INT PIN IS OFF\n");

		NET_REG_ADDR = DM9000_REG_MWCMD;
		for(i = 0; i < (0x0100 / 2); i++)
		{
			if(i < 3)
				NET_REG_DATA = 0xffff;
			else
				NET_REG_DATA = i * 0x0101;
		}

		printk("DM9K_DEBUG ==> PACKET IS SEND \n");
		iow(DM9000_REG_TCR, DM9000_TCR_SET);

		while(ior(DM9000_REG_TCR) & DM9000_TCR_SET) dm9k_udelay (5);
		if(ior(DM9000_REG_ISR) & DM9000_TX_INTR)
			printk("DM9K_DEBUG ==> INT PIN IS ACTIVE \n");
		else
			printk("DM9K_DEBUG ==> INT PIN IS NOT ACTIVE \n");

		for(i = 0; i < 10; i++)
			dm9k_udelay(1000);

	}while(1);
}

/*******************************************************************************
*	函数名: etherdev_init
*	参  数: 无
*	返  回: 无
*	功  能: uIP 接口函数,初始化网卡
*/
void etherdev_init(void)
{
//	INT8U err;
	if(dm9000_rx_sem==NULL)
	//if(dm9000_rx_mutex==NULL)
	{
		dm9000_rx_sem=OSSemCreate(0);
		//dm9000_rx_mutex=OSMutexCreate(0,&err);
		//OSMutexPend(dm9000_rx_mutex,0, &err);
	}
	//if(dm9000_tx_sem==NULL)
	//{
	//	dm9000_tx_sem=OSSemCreate(0);
	//}
	DM9K_CtrlLinesConfig();
	DM9K_FSMCConfig();

	s_FSMC_Init_Ok = 1;

	dm9k_initnic();
#ifdef Rx_Int_enable
	DM9K_IntModeInit();
#endif
}

/*******************************************************************************
*	函数名: etherdev_send
*	参  数: p_char : 数据缓冲区
*			length : 数据长度
*	返  回: 无
*	功  能: uIP 接口函数,发送一包数据
*/
void etherdev_send(uint8_t *p_char, uint16_t length)
{
	dm9k_Tx(p_char, length);
}

extern OS_EVENT *uIP_Mutex;

uint16_t etherdev_receive(uint8_t *p_char)
{
	INT8U err;
	uint16_t len=0;
//	static uint16_t flag=0;
//	uint8_t  save_reg;

#ifdef Rx_Int_enable
	//len=dm9k_receive_packet(p_char);
	//if(len)return len;
	//OSSemPend(dm9000_rx_sem, 1, &err);
	OSSemPend(dm9000_rx_sem, 0, &err);
	//OSMutexPend(dm9000_rx_mutex,10000, &err);
	if(err==OS_NO_ERR)
	{
		OSMutexPend(uIP_Mutex,0,&err);
		//OSSemPend(dm9000_rx_sem, 1, &err);
		//OSSemPend(dm9000_rx_sem, OS_TICKS_PER_SEC/100, &err);
		//OSSemPend(dm9000_rx_sem, OS_TICKS_PER_SEC/10, &err);
		//if(err==OS_TIMEOUT)return 0;
		//return g_RxLen;
		//save_reg = NET_REG_ADDR;
		iow(DM9000_REG_IMR , DM9000_IMR_OFF);				/* 关闭DM9000A 中断 */
		len=dm9k_Rx(p_char);
		//iow(DM9000_REG_RCR, DM9000_RCR_SET);
		iow(DM9000_REG_IMR , DM9000_IMR_SET);				/* 开启 DM9000A 中断 */
		//NET_REG_ADDR = save_reg;

		if(len==0)
		{
//			flag=0;
			OSMutexPost(uIP_Mutex);
		}
		//else flag=1;
		//NET_REG_ADDR = save_reg;
		//iow(DM9000_REG_IMR , DM9000_IMR_SET);				/* 开启 DM9000A 中断 */
	}

	return len;
#else
	return dm9k_Rx(p_char);
#endif
}

/*******************************************************************************
*	函数名: etherdev_chkmedia
*	参  数: p_char : 数据缓冲区
*			length : 数据长度
*	返  回: 无
*	功  能: uIP 接口函数, 检测网络连接状态
*/
void etherdev_chkmedia(void)
{
//	uint8_t status;

	while(!(ior(DM9000_REG_NSR) & DM9000_PHY))
	{
		dm9k_udelay(2000);
	}
}


/*******************************************************************************
*	函数名: etherdev_poll
*	参  数: 无
*	返  回: 无
*	功  能: uIP 接口函数, 采用查询方式接收一个IP包
*/
/*
                              etherdev_poll()

    This function will read an entire IP packet into the uip_buf.
    If it must wait for more than 0.5 seconds, it will return with
    the return value 0. Otherwise, when a full packet has been read
    into the uip_buf buffer, the length of the packet is returned.
*/
uint16_t etherdev_poll(void)
{
	uint16_t bytes_read = 0;
#if 0

	/* tick_count threshold should be 12 for 0.5 sec bail-out
		One second (24) worked better for me, but socket recycling
		is then slower. I set UIP_TIME_WAIT_TIMEOUT 60 in uipopt.h
		to counter this. Retransmission timing etc. is affected also. */
	while ((!(bytes_read = etherdev_read())) && (timer0_tick() < 12)) continue;

	timer0_reset();

#endif
	return bytes_read;
}

/*******************************************************************************
*	函数名: dm9k_ReadID
*	参  数: 无
*	返  回: 无
*	功  能: 读取芯片ID
*/
uint32_t dm9k_ReadID(void)
{
	uint8_t vid1,vid2,pid1,pid2;

	if (s_FSMC_Init_Ok == 0)
	{
		DM9K_CtrlLinesConfig();
		DM9K_FSMCConfig();

		s_FSMC_Init_Ok = 1;
	}
	vid1 = ior(DM9000_REG_VID_L) & 0xFF;
	vid2 = ior(DM9000_REG_VID_H) & 0xFF;
	pid1 = ior(DM9000_REG_PID_L) & 0xFF;
	pid2 = ior(DM9000_REG_PID_H) & 0xFF;

	return (vid2 << 24) | (vid1 << 16) | (pid2 << 8) | pid1;
}

/*******************************************************************************
*	函数名: DM9000AE_CtrlLinesConfig
*	参  数: 无
*	返  回: 无
*	功  能: 配置DM9000AE控制口线，FSMC管脚设置为复用功能
*/
static void DM9K_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能 FSMC, GPIOD, GPIOE, GPIOF, GPIOG 和 AFIO 时钟 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
	                     RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG |
	                     RCC_APB2Periph_AFIO, ENABLE);

	/* 设置 PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
	 PD.10(D15), PD.14(D0), PD.15(D1) 为复用推挽输出 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
	                            GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |
	                            GPIO_Pin_15; // | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* 设置 PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
	 PE.14(D11), PE.15(D12) 为复用推挽输出 */
	/* PE3,PE4 用于A19, A20, STM32F103ZE-EK(REV 2.0)必须使能 */
	/* PE5,PE6 用于A21, A22, STM32F103ZE-EK(REV 1.0)必须使能 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
	                            GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
	                            GPIO_Pin_15 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* 设置 PF2(A2))  为复用推挽输出 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/* 设置 PG.12(NE4 ) 为复用推挽输出  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}

/*******************************************************************************
*	函数名: DM9K_FSMCConfig
*	参  数: 无
*	返  回: 无
*	功  能: 配置FSMC并口访问时序
*/
static void DM9K_FSMCConfig(void)
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTimingInitStructure;

	/*-- FSMC Configuration ------------------------------------------------------*/
	/*----------------------- SRAM Bank 4 ----------------------------------------*/
	/* FSMC_Bank1_NORSRAM4 configuration */
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 0;
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0;
	FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 4;
	FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0;
	FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0;
	FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0;
	FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;

	/* Color LCD configuration ------------------------------------
	 LCD configured as follow:
	    - Data/Address MUX = Disable
	    - Memory Type = SRAM
	    - Data Width = 16bit
	    - Write Operation = Enable
	    - Extended Mode = Enable
	    - Asynchronous Wait = Disable */
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

	/* - BANK 4 (of NOR/SRAM Bank 0~3) is enabled */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

void DM9K_Dump(void)
{
	uint8_t save;
//	uint8_t i,j,k;
	save = NET_REG_ADDR;
	//printf("IMR:%02x\nISR:%02x\n",ior(DM9000_REG_IMR),ior(DM9000_REG_ISR));
	//printf("TSR1:%02x\nTSR2:%02x\n",ior(DM9000_REG_TSR1),ior(DM9000_REG_TSR2));
	//printf("RSR:%02x\nROCR:%02x\n",ior(DM9000_REG_RSR),ior(DM9000_REG_ROCR));
	printf("Event:%d,%d\n",g_MaxEvent,g_CurEvent);
#if 0
	printf("DM9Kregs:\n");
	for(i=0,k=0;;)
	{
		for(j=0;j<16;j++,i++)printf("%02x ",ior(i));
		printf("\n");
		k++;
		if(k==256/16)break;
	}
	printf("\n");
#endif
	NET_REG_ADDR = save;
}
