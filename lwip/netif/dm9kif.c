
/****************************************************************************
* Copyright (C), 2009-2010, www.armfly.com  ����������
*
* ���������ڰ�����STM32F103ZE-EK�������ϵ���ͨ��             ��
* ��QQ: 1295744630, ������armfly, Email: armfly@qq.com       ��
*
* �ļ���: dm9k_uip.c
* ���ݼ���: Davicom DM9000A uP NIC fast Ethernet driver for uIP.
*
* �ļ���ʷ:
* �汾��  ����       ����    ˵��
* v0.1    2010-01-18 armfly  �������ļ�
*
*/
//#define DM9000A_FLOW_CONTROL
//#define DM9000A_UPTO_100M
//#define Fifo_Point_Check
//#define Point_Error_Reset
#define Fix_Note_Address
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32_eval.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_fsmc.h"
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/stats.h"
#include "lwip/err.h"
#include "lwip/debug.h"

#include "netif/etharp.h"
#include "netif/dm9kif.h"
#include "ucos_ii.h"
#include "flash.h"
#include "printf_level.h"
#include "dtimer.h"

/* DM9000A ���պ������ú� */
#define Dm9k_Int_Enable
#define Max_Int_Count			1
//#define Max_Ethernet_Lenth		1536
#define Broadcast_Jump
#define Max_Broadcast_Lenth		500

#define DM9000_PKT_RDY		0x01	/* Packet ready to receive */
#define DM9000_PKT_MAX		1536	/* Received packet max size */

/* Define those to better describe your network interface. */
#define IFNAME0 'd'
#define IFNAME1 'm'

struct ethernetif {
  struct eth_addr *ethaddr;
  /* Add whatever per-interface state that is needed here. */
};

//static const struct eth_addr ethbroadcast = {{0xff,0xff,0xff,0xff,0xff,0xff}};
//extern struct netif *dm9k_netif;

/* DM9000A ���ͺ������ú� */
#define Max_Send_Pack			2

/* ϵͳȫ������ú�
#undef    uint8_t
#define   uint8_t				unsigned char
#undef    uint16_t
#define   uint16_t				unsigned short
#undef    UINT32
#define   UINT32				unsigned long
*/

#define   NET_BASE_ADDR		0x6C100000
#define   NET_REG_ADDR		(*((volatile uint16_t *) NET_BASE_ADDR))
#define   NET_REG_DATA		(*((volatile uint16_t *) (NET_BASE_ADDR + 8)))

//#define FifoPointCheck

/*=============================================================================
  ϵͳȫ��ı���
  =============================================================================*/
struct netif *dm9k_netif=NULL;

OS_EVENT *dm9000_rx_sem=NULL;
OS_EVENT *dm9000_rx_mutex=NULL;
OS_EVENT *dm9000_tx_sem=NULL;
static unsigned char DEF_MAC_ADDR[NETIF_MAX_HWADDR_LEN] =
	//{0x00, 0x60, 0x6e, 0x90, 0x00, 0xae};
	{0x00, 0xE1, 0x6E, 0x00, 0x00, 0x01};
uint16_t  TxPktCnt = 0;
uint16_t  RxPktCnt = 0;
uint16_t g_RxLen=0;
uint16_t queue_pkt_len=0;
uint16_t g_TxComplete=1;
uint16_t g_RxFlag=0;
uint16_t g_MaxEvent=0;
uint16_t g_CurEvent=0;
uint16_t TxIntCnt = 0;

uint8_t s_FSMC_Init_Ok = 0;	/* ����ָʾFSMC�Ƿ��ʼ�� */

struct pbuf *txq_head = NULL,*tx_tail = NULL;
uint32_t dm9k_in_interrupt=0;

extern struct netif *dm9k_netif;

#define printk(...)	printf(__VA_ARGS__)
//#define printk printf

void dm9k_debug_test(void);
void dm9k_udelay(uint16_t time);

static void DM9K_CtrlLinesConfig(void);
static void DM9K_FSMCConfig(void);
static void DM9K_IntModeInit(void);
void dm9k_reset(void);
void low_level_init(struct netif *netif);
static err_t ethernetif_output(struct netif *netif, struct pbuf *p, struct ip_addr *ipaddr);
static void ethernetif_input(struct netif *netif);



/*******************************************************************************
*	������: dm9k_udelay
*	��  ��: time �� �ӳ�ʱ�䣬����ȷ��us����
*	��  ��: ��
*	��  ��: �ӳٺ���
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
*	������: ior
*	��  ��: reg ���Ĵ�����ַ
*	��  ��: ��
*	��  ��: �����Ĵ�����ֵ
*/
__INLINE uint8_t ior(uint8_t reg)
{
	NET_REG_ADDR = reg;
	return (NET_REG_DATA);
}

/*******************************************************************************
*	������: iow
*	��  ��: reg ���Ĵ�����ַ
*			writedata : д�������
*	��  ��: ��
*	��  ��: дDM9000AE�Ĵ�����ֵ
*/
__INLINE void iow(uint8_t reg, uint8_t writedata)
{
	NET_REG_ADDR = reg;
	NET_REG_DATA = writedata;
}

/*******************************************************************************
*	������: dm9k_phy_read
*	��  ��: phy_reg �� �Ĵ�����ַ
*	��  ��: ��ȡ������
*	��  ��: дDM9000A PHY �Ĵ���
*/
uint16_t dm9k_phy_read(uint8_t reg)
{
	//unsigned long flags;
	uint16_t reg_save;
	uint16_t ret;


	/* Save previous register address */
	reg_save = NET_REG_ADDR;

	/* Fill the phyxcer register into REG_0C */
	iow(DM9000_REG_EPAR, DM9000_PHY | reg);

	//iow(DM9000_REG_EPCR, EPCR_ERPRR | EPCR_EPOS);	/* Issue phyxcer read command */
	iow(DM9000_REG_EPCR, (1<<2) | (1<<3));	/* Issue phyxcer read command */

	NET_REG_ADDR = reg_save;
	uDelay(1000);
	//dm9000_msleep(db, 1);		/* Wait read complete */
	reg_save = NET_REG_ADDR;
	while(ior(DM9000_REG_EPCR) & 0x01);

	iow(DM9000_REG_EPCR, 0x0);	/* Clear phyxcer read command */

	/* The read data keeps on REG_0D & REG_0E */
	ret = (ior(DM9000_REG_EPDRH) << 8) | ior(DM9000_REG_EPDRL);

	/* restore the previous address */
	NET_REG_ADDR = reg_save;

	return ret;
}
/*******************************************************************************
*	������: dm9k_phy_write
*	��  ��: phy_reg �� �Ĵ�����ַ
*			writedata �� д�������
*	��  ��: ��
*	��  ��: дDM9000A PHY �Ĵ���
*/
void dm9k_phy_write(uint8_t phy_reg, uint16_t writedata)
{
	/* ����д�� PHY �Ĵ�����λ�� */
	iow(DM9000_REG_EPAR, phy_reg | DM9000_PHY);

	/* ����д�� PHY �Ĵ�����ֵ */
	iow(DM9000_REG_EPDRH, ( writedata >> 8 ) & 0xff);
	iow(DM9000_REG_EPDRL, writedata & 0xff);

	iow(DM9000_REG_EPCR, 0x0a); 						/* ������д�� PHY �Ĵ��� */
	while(ior(DM9000_REG_EPCR) & 0x01);					/* ��Ѱ�Ƿ�ִ�н��� */
	iow(DM9000_REG_EPCR, 0x08); 						/* ���д������ */
}

#if 0
/*******************************************************************************
*	������: dm9k_receive_packet
*	��  ��: _uip_buf : ���ջ�����
*	��  ��: > 0 ��ʾ���յ����ݳ���, 0��ʾû������
*	��  ��: ��ȡһ������
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
	//iow(DM9000_REG_IMR , DM9000_IMR_OFF);				/* �ر� DM9000A �ж� */
	//save_reg = NET_REG_ADDR;
	
//	ReceiveLength = 0;								/* ������յĳ��� */
	ReceiveData = (uint16_t *)_uip_buf;
//	jump_packet = 0;								/* ����������� */
	ior(DM9000_REG_MRCMDX);							/* ��ȡ�ڴ����ݣ���ַ������ */
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
	/* �����ڴ�����λ�� */
	//calc_MRR = (((uint16_t)ior(DM9000_REG_MRRH)) << 8) + ior(DM9000_REG_MRRL);

	GoodPacket = TRUE;
	/* ��ȡ��������Ѷ �� ���� */
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
		/* ��ʼ���ڴ�����ϰᵽ��ϵͳ�У�ÿ���ƶ�һ�� word */
		for(i = 0 ; i < calc_len ; i++)
			ReceiveData[i] = NET_REG_DATA;

	} else {
		/* need to dump the packet's data */
		for(i = 0 ; i < calc_len ; i++)
			rx_status = NET_REG_DATA;
		return 0;
	}
	do{
		ior(DM9000_REG_MRCMDX);							/* ��ȡ�ڴ����ݣ���ַ������ */
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
	//iow(DM9000_REG_IMR , DM9000_IMR_SET);				/* ���� DM9000A �ж� */
	return rx_length-4;
}

/*******************************************************************************
*	������: dm9k_send_packet
*	��  ��: p_char : �������ݻ�����
*			length : ���ݳ���
*	��  ��: ����ʧ�ܷ���FALSE�����ͳɹ�����TRUE
*	��  ��: ����һ������
*/
bool dm9k_Tx(uint8_t *p_char, uint16_t length)
{
	uint16_t calc_len,i;
	uint16_t *SendData = (uint16_t *) p_char;
	if(TxPktCnt>1)return FALSE;
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
	return TRUE;
	//iow(DM9000_REG_IMR,DM9000_IMR_SET);
}
#endif

void dm9k_Tx_Done(void)
{
	uint8_t status=ior(DM9000_REG_NSR);
#if 0
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
#else
	if(status & NSR_TX1END)
	{
		if(TxPktCnt)TxPktCnt--;
	}
	if(status & NSR_TX2END)
	{
		if(TxPktCnt)TxPktCnt--;
	}
	if(TxPktCnt>0)
	{
		iow(DM9000_REG_TXPLL,queue_pkt_len & 0xff);
		iow(DM9000_REG_TXPLH,queue_pkt_len >> 8);
		iow(DM9000_REG_TCR,DM9000_TCR_SET);
	}
#endif
}

void dm9k_copyout(uint8_t *buf, uint16_t len)
{
	u16_t count=len>>1;
	if(((u32_t)buf) & 0x01)
	{								/* ��Ϊ������ַ���ֽڶ�ȡ*/
		while(count--) {
			NET_REG_DATA = (*buf) | (((u16_t)*(buf + 1)) << 8);
			buf += 2;
		}
		if (len & 0x01)         	/* If odd length, do last byte */
			NET_REG_DATA = *buf;
	}
	else
	{								/* ��Ϊż����ַ�򰴰��ֶ�ȡ */
		u16_t *dataw;
		dataw = (u16_t *)buf;
		while(count--)
			NET_REG_DATA = *dataw++;
		if (len & 0x01)            	/* If odd length, do last byte */
			NET_REG_DATA = *(u8_t *)dataw;
	}
}

void dm9k_outpad(uint16_t len)
{
	u16_t count;

	count = len >> 1;
	while(count--)
		NET_REG_DATA = 0;
}

void dm9k_copyin(uint8_t *buf, uint16_t len)
{
	u16_t count;
	u16_t *dataw;

	count =  len >> 1;

	dataw = (uint16_t *)buf;				// Use pointer for speed 
	while(count--)                      		// Get words 
		*dataw++ = NET_REG_DATA;
	if (len & 0x01)                        		// If odd length, do last byte 
		*(uint8_t *)dataw = NET_REG_DATA;
}
void dm9k_discard( uint16_t len)
{
	//u16_t tmpw;
	u16_t count;

	count = (len + 1) >> 1;

	while(count--)                      		
		len = NET_REG_DATA;
}

/*******************************************************************************
*	������: dm9k_ISR
*	��  ��: ��
*	��  ��: ��
*	��  ��: �жϴ����� 
*/
void  dm9k_ISR(void)
{
	uint8_t  save_reg;
	uint8_t  isr_status;
	save_reg = NET_REG_ADDR;							/* �ݴ���ʹ�õ�λ�� */
	//iow(DM9000_REG_IMR , DM9000_IMR_OFF);				/* �ر� DM9000A �ж� */
	dm9k_in_interrupt++;

	isr_status = ior(DM9000_REG_ISR);					/* ȡ���жϲ���ֵ */
	iow(DM9000_REG_ISR,isr_status);						/* ���жϱ�־ */
	
	//printf_0("dm9k:%02x\n",isr_status);

	if(isr_status & DM9000_OVERFLOW_INTR)
	{
		//dm9k_reset();
		low_level_init(dm9k_netif);
		return;
	}
#if 0
	if(isr_status & DM9000_LINK_CHANG)
	{
		//isr_status &= ~DM9000_LINK_CHANG;
		process_link_change();
	}
#endif
	if(isr_status & DM9000_RX_INTR) 					/* ����Ƿ�Ϊ�����ж� */
	{
		ethernetif_input(dm9k_netif);
	}
	if(isr_status & DM9000_TX_INTR) 					/* ����Ƿ�Ϊ�����ж� */
	{
		//iow(DM9000_REG_ISR, DM9000_TX_INTR);			/*������жϱ�־*/	
		dm9k_Tx_Done();
#if 0
		if(TxIntCnt)TxIntCnt--;
		//if(dm9000_tx_sem->OSEventCnt==0)OSSemPost(dm9000_tx_sem);
		if(dm9000_tx_sem->OSEventGrp==0)
		{
			//dm9000_tx_sem->OSEventCnt=0;
			//dm9000_tx_sem->OSEventGrp=1;
			OSSemPost(dm9000_tx_sem);
		}
#endif
		/*
		if(TxIntCnt)TxIntCnt--;
		else
		{
			OSSemPost(dm9000_tx_sem);
		}
		*/
	}

	dm9k_in_interrupt--;
	//iow(DM9000_REG_IMR , DM9000_IMR_SET);				/* ���� DM9000A �ж� */
	NET_REG_ADDR = save_reg;							/* �ָ���ʹ�õ�λ�� */
}

/*
 * low_level_output():
 *
 * Should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 */


static err_t
low_level_output(struct netif *netif, struct pbuf *p)
{
	struct pbuf *q;

	u8_t chain;
	u8_t * tr_ptr;
	u16_t tr_len, temp_dw;
	u16_t packetLength;
	u32_t count;
	//u16_t padLength=0;
//	INT8U err;
#if OS_CRITICAL_METHOD == 3
	OS_CPU_SR  cpu_sr;
#endif

	/* Set up to transfer the packet contents to the NIC RAM. */
	packetLength = p->tot_len;

	/* packetLength muse >=64 (see 802.3)
	if (packetLength < 64)
	{
		padLength = 64 - packetLength;
		packetLength = 64;
	} */

	//iow(DM9000_REG_IMR,DM9000_IMR_OFF);
#if 0
	if(!dm9k_in_interrupt)
	{
	#if 0	/*ѭ���ȴ����嵽���ÿ���*/
		while(TxPktCnt>1);
	#else	/*�������е��ã��ȴ�����������Ϊֹ*/
		while(TxPktCnt>1)	/**/
		{
			OSSemPend(dm9000_tx_sem, 0, &err);
		}
	#endif
	}
	else
	{
		//if(TxPktCnt<2)TxIntCnt++;
		//else return ERR_USE;
		
		if(TxPktCnt>1)
		{
			while(!(ior(DM9000_REG_ISR) & DM9000_TX_INTR));
			iow(DM9000_REG_ISR,DM9000_TX_INTR);
			dm9k_Tx_Done();
		}
		TxIntCnt++;
		//if(TxPktCnt>1)return ERR_USE;		/*���ж��е��ã���������*/
	}
#else
	if(!dm9k_in_interrupt)
	{
		OS_ENTER_CRITICAL();
		count = 0;	
		while(TxPktCnt>1)
		{
			count++;
			if(count > 65536)
			{
				count = 0;	
				dm9k_Tx_Done();
			}
		}
		//NVIC_DisableIRQ(EXTI1_IRQn);
	}
	else
	{
		if(TxPktCnt>1)
		{
			//NVIC_EnableIRQ(EXTI1_IRQn);
			//return ERR_USE;
			return ERR_OK;
		}
	}
#endif

	NET_REG_ADDR=DM9000_REG_MWCMD;

	/* write packet to ring buffers. */
	for(q = p, chain = 0; q != NULL; q = q->next)
	{
		if(chain == 1)
		{
			if(((q->len-1) & 0x01) && (q->next != NULL))
			{
				tr_len = q->len - 2;
				tr_ptr = ((u8_t*)q->payload) + 1;

				temp_dw = *(((u8_t *)q->payload) + q->len - 1);
				temp_dw += *(u8_t *)(q->next->payload) << 8;

				chain = 1;
			}
			else
			{
				tr_len = q->len - 1;
				tr_ptr = ((u8_t*)q->payload) + 1;
				chain = 0;
			}
		}
		else
		{
			if((q->len & 0x01) && (q->next != NULL))
			{
				tr_len = q->len - 1;
				tr_ptr = (u8_t*)q->payload;

				temp_dw = *(((u8_t *)q->payload) + q->len - 1);
				temp_dw += *(u8_t *)(q->next->payload) << 8;

				chain = 1;
			}
			else
			{
				tr_len = q->len;
				tr_ptr = (u8_t*)q->payload;

				chain = 0;
			}
		}

		//ne2k_copyout(tr_len, tr_ptr);
		dm9k_copyout(tr_ptr, tr_len);

		if (chain == 1) NET_REG_DATA = temp_dw;	
	}
	//if(padLength)dm9k_outpad(padLength);

	if(TxPktCnt==0)
	{
		TxPktCnt++;
		iow(DM9000_REG_TXPLL,packetLength & 0xff);
		iow(DM9000_REG_TXPLH,packetLength >> 8);
		iow(DM9000_REG_TCR,DM9000_TCR_SET);
	}
	else
	{
		TxPktCnt++;
		queue_pkt_len = packetLength;
	}

	if(!dm9k_in_interrupt)
	{
		//iow(DM9000_REG_IMR,DM9000_IMR_SET);
		//if(TxPktCnt < 2)OSSemPost(dm9000_tx_sem);
		//OSSemPost(dm9000_tx_sem);
		//NVIC_EnableIRQ(EXTI1_IRQn);
		OS_EXIT_CRITICAL();
	}
	/*
	else
	{
		while(!(ior(DM9000_REG_ISR) & DM9000_TX_INTR));
		iow(DM9000_REG_ISR,DM9000_TX_INTR);
		dm9k_Tx_Done();
		//if(TxPktCnt < 2)OSSemPost(dm9000_tx_sem);
		//OSSemPost(dm9000_tx_sem);
	}
	*/
#ifdef LINK_STATS
	lwip_stats.link.xmit++;
#endif /* LINK_STATS */      

	return ERR_OK;
}

/*
 * low_level_input():
 *
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 */
volatile int dm9k_err_cnt = 0;

static struct pbuf *
low_level_input(struct netif *netif)
{
	struct pbuf *p, *q;
	u16_t packetLength, len;
	u8_t PDHeader[18];   /* Temp storage for ethernet headers */
	u8_t * payload;
	uint8_t  rx_checkbyte;
	//uint16_t rx_status, rx_length;

	ior(DM9000_REG_MRCMDX);	/* ��ȡ�ڴ����ݣ���ַ������ */
	//NET_REG_ADDR = DM9000_REG_MRCMDX;
	rx_checkbyte = NET_REG_DATA;			/*  */
	if (rx_checkbyte > DM9000_PKT_RDY) {
		printk("status check failed: %d\n", rx_checkbyte);
		iow(DM9000_REG_RCR,0x00);	/* Stop Device */
		iow(DM9000_REG_ISR,IMR_PAR);/* Stop INT request */
		return NULL;
	}
	if(rx_checkbyte != DM9000_PKT_RDY)
		return NULL;
	/* ��ȡ��������Ѷ �� ���� */
	NET_REG_ADDR = DM9000_REG_MRCMD;
	//rx_status = NET_REG_DATA;
	//rx_length = NET_REG_DATA;

	/* get the first 18 bytes from nic */
	dm9k_copyin(PDHeader,18);

	/* Store real length, set len to packet length - header */
	packetLength = ((unsigned) PDHeader[2] | (PDHeader[3] << 8 ));

	/* verify if the packet is an IP packet or ARP packet */
	if((PDHeader[3]>0x06)||(PDHeader[16] != 8)||(PDHeader[17] != 0 && PDHeader[17] != 6))
	{
		dm9k_discard(packetLength-14);
		return NULL;
	}  

	/* We allocate a pbuf chain of pbufs from the pool. */
	p = pbuf_alloc(PBUF_RAW, packetLength, PBUF_POOL);

	if (p != NULL) {
		/* We iterate over the pbuf chain until we have read the entire
		packet into the pbuf. */

		/* This assumes a minimum pbuf size of 14 ... a good assumption */
		memcpy(p->payload, PDHeader + 4, 14);   

		for(q = p; q != NULL; q = q->next) {
			/* Read enough bytes to fill this pbuf in the chain. The
			 available data in the pbuf is given by the q->len
			 variable. */
			payload = q->payload;
			len = q->len;
			if (q == p) {
				payload += 14;
				len -=14;
			}

			dm9k_copyin(payload,len);
		}

		if(p->tot_len > 100)
			dm9k_err_cnt = (int)p->payload;
#ifdef LINK_STATS
		lwip_stats.link.recv++;
#endif /* LINK_STATS */      
	}else{
		/* no more PBUF resource, Discard packet in buffer. */	
		dm9k_discard(packetLength-14);
		printk("D");
#ifdef LINK_STATS
		lwip_stats.link.memerr++;
		lwip_stats.link.drop++;
#endif /* LINK_STATS */      
	}

	return p;  
}

/*******************************************************************************
*	������: dm9k_debug_test
*	��  ��: ��
*	��  ��: ��
*	��  ��: ����DM9000AE�ĺ���,�����Ŵ�
*/
void dm9k_debug_test(void)
{
	uint32_t check_device;
	uint8_t  check_iomode;
	uint8_t  check_reg_fail = 0;
	uint8_t  check_fifo_fail = 0;
	uint16_t i;
	uint16_t j;

	iow(DM9000_REG_NCR, DM9000_REG_RESET);			/* �� DM9000A ����������� */
	dm9k_udelay(10);								/* delay 10us */
	iow(DM9000_REG_NCR, DM9000_REG_RESET);			/* �� DM9000A ����������� */
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

#if 0
/*******************************************************************************
*	������: etherdev_send
*	��  ��: p_char : ���ݻ�����
*			length : ���ݳ���
*	��  ��: ��
*	��  ��: uIP �ӿں���,����һ������
*/
void etherdev_send(uint8_t *p_char, uint16_t length)
{
	dm9k_Tx(p_char, length);
}

//extern OS_EVENT *uIP_Mutex;
uint16_t etherdev_receive(uint8_t *p_char)
{
	INT8U err;
	uint16_t len=0;
//	static uint16_t flag=0;
//	uint8_t  save_reg;

#ifdef Dm9k_Int_Enable
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
		iow(DM9000_REG_IMR , DM9000_IMR_OFF);				/* �ر�DM9000A �ж� */
		len=dm9k_Rx(p_char);
		//iow(DM9000_REG_RCR, DM9000_RCR_SET);
		iow(DM9000_REG_IMR , DM9000_IMR_SET);				/* ���� DM9000A �ж� */
		//NET_REG_ADDR = save_reg;

		if(len==0)
		{
//			flag=0;
			OSMutexPost(uIP_Mutex);
		}
		//else flag=1;
		//NET_REG_ADDR = save_reg;
		//iow(DM9000_REG_IMR , DM9000_IMR_SET);				/* ���� DM9000A �ж� */
	}

	return len;
#else
	return dm9k_Rx(p_char);
#endif
}
#endif

/*******************************************************************************
*	������: etherdev_chkmedia
*	��  ��: p_char : ���ݻ�����
*			length : ���ݳ���
*	��  ��: ��
*	��  ��: uIP �ӿں���, �����������״̬
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
*	������: etherdev_poll
*	��  ��: ��
*	��  ��: ��
*	��  ��: uIP �ӿں���, ���ò�ѯ��ʽ����һ��IP��
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
*	������: dm9k_ReadID
*	��  ��: ��
*	��  ��: ��
*	��  ��: ��ȡоƬID
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

	return (((uint32_t)vid2) << 24) | (((uint32_t)vid1) << 16) | (((uint32_t)pid2) << 8) | pid1;
}

/*******************************************************************************
*	������: DM9000AE_CtrlLinesConfig
*	��  ��: ��
*	��  ��: ��
*	��  ��: ����DM9000AE���ƿ��ߣ�FSMC�ܽ�����Ϊ���ù���
*/
static void DM9K_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ�� FSMC, GPIOD, GPIOE, GPIOF, GPIOG �� AFIO ʱ�� */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
	                     RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG |
	                     RCC_APB2Periph_AFIO, ENABLE);

	/* ���� PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
	 PD.10(D15), PD.14(D0), PD.15(D1) Ϊ����������� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
	                            GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |
	                            GPIO_Pin_15; // | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* ���� PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
	 PE.14(D11), PE.15(D12) Ϊ����������� */
	/* PE3,PE4 ����A19, A20, STM32F103ZE-EK(REV 2.0)����ʹ�� */
	/* PE5,PE6 ����A21, A22, STM32F103ZE-EK(REV 1.0)����ʹ�� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
	                            GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
	                            GPIO_Pin_15 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* ���� PF2(A2))  Ϊ����������� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/* ���� PG.12(NE4 ) Ϊ�����������  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}

/*******************************************************************************
*	������: DM9K_FSMCConfig
*	��  ��: ��
*	��  ��: ��
*	��  ��: ����FSMC���ڷ���ʱ��
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

/*******************************************************************************
*	������: DM9K_IntModeInit
*	��  ��: ��
*	��  ��: ��
*	��  ��: �����ж�ģʽ��ʼ��
*/
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
*	������: dm9k_hash_table
*	��  ��: ��
*	��  ��: ��
*	��  ��: ���� DM9000A MAC �� �㲥 �� �ಥ �Ĵ���
*/
#define DM9000_USER_PAD
void dm9k_hash_table(struct netif *netif)
{
	uint8_t i,mac[6];
	volatile unsigned char *pCPUID;
	unsigned rand_num,rand0=0;
  /* set MAC hardware address length */

	//memcpy(mac, "\x00\xe1\x6e\0\0\x2",6);
	//Flash_Write(FLASH_DM9000_MAC, mac, 6);
#ifdef DM9000_USER_PAD
	Flash_Read(mac, FLASH_DM9000_MAC, 6);
	for(i=0;i<3;i++)
	{
		if(mac[i] != DEF_MAC_ADDR[i])break;
	}
#if 0
	if(i==3)
	{
		for(;i<6;i++)
		if(mac[i] != '\0' && mac[i] != '\xff')break;
		if(i<6)memcpy(DEF_MAC_ADDR, mac, 6);
	}
#else
	pCPUID = (unsigned char*)0x1ffff7e8;
	i=3;
	for(;i<6;i++,pCPUID++)
	{
		//srand(*pCPUID | (*(pCPUID+3)<<8) | (*(pCPUID+6)<<16) | (*(pCPUID+9)<<24));
		rand_num = *pCPUID | (*(pCPUID+3)<<8) | (*(pCPUID+6)<<16) | (*(pCPUID+9)<<24);
		//printf("srand_num:%08x\n",rand_num);
		srand(rand_num);
		srand(rand());
		//srand(rand());
		rand_num=rand()+rand0;
		rand0 = rand();
		//printf("rand_num:%08x\n",rand_num);
		DEF_MAC_ADDR[i]=(rand_num ^ (rand_num>>8) ^ (rand_num>>16) ^ (rand_num>>24))&0xff;
	}
	DEF_MAC_ADDR[5] += ~*(unsigned char *)Flash_Read(mac,FLASH_DM9000_MAC+6,1);
#endif
#endif
	netif->hwaddr_len = 6;

		
  /* set MAC hardware address */
	for (i = 0; i < 6; i++)
	{
	#ifndef DM9000_USER_PAD
		//DEF_MAC_ADDR[i] = ior(DM9000_REG_PAR + i);
		DEF_MAC_ADDR[i] = ior(DM9000_REG_PAR + i);
	#endif
		netif->hwaddr[i] = DEF_MAC_ADDR[i];
	}
	printf("MAC:");
	for(i = 0; i < 5; i++)
		printf("%02X-",DEF_MAC_ADDR[i]);
	printf("%02X\n",DEF_MAC_ADDR[i]);
	
  /* maximum transfer unit */
	netif->mtu = 1500;
	//netif->mtu = DM9000_PKT_MAX;
	
  /* broadcast capability */
	netif->flags = NETIF_FLAG_BROADCAST;
	/* ���� ���� MAC λ�ã������ MyHardware */
	#ifdef DM9000_USER_PAD
	for(i = 0; i < 6; i++)
		iow(DM9000_REG_PAR + i, DEF_MAC_ADDR[i]);
	#endif

	for(i = 0; i < 8; i++) 								/* ��� �����ಥ���� */
		iow(DM9000_REG_MAR + i, 0x00);
		//iow(DM9000_REG_MAR + i, 0xFF);
	iow(DM9000_REG_MAR + 7, 0x80);  					/* ������ �㲥�� ���� */
}

/*******************************************************************************
*	������: dm9k_reset
*	��  ��: ��
*	��  ��: ��
*	��  ��: ��DM9000AE���������λ
*/
void dm9k_reset(void)
{
	//iow(DM9000_REG_NCR, DM9000_REG_RESET);			/* �� DM9000A ����������� */
	//dm9k_udelay(10);								/* delay 10us */
	iow(DM9000_REG_NCR, DM9000_REG_RESET);			/* �� DM9000A ����������� */
	dm9k_udelay(10);								/* delay 10us */

	/* �����Ǵ���������� */
	iow(DM9000_REG_IMR, DM9000_IMR_OFF); 			/* �����ڴ��Ի�ģʽ */
	iow(DM9000_REG_TCR2, DM9000_TCR2_SET);			/* ���� LED ��ʾģʽ1:ȫ˫��������˫���� */

	/* ���������Ѷ */
	iow(DM9000_REG_NSR, 0x2c);
	iow(DM9000_REG_TCR, 0x00);
	iow(DM9000_REG_ISR, 0x3f);

#ifdef DM9000A_FLOW_CONTROL
	iow(DM9000_REG_BPTR, DM9000_BPTR_SET);			/* ��˫���������� */
	iow(DM9000_REG_FCTR, DM9000_FCTR_SET);			/* ȫ˫���������� */
	iow(DM9000_REG_FCR, DM9000_FCR_SET);			/* ������������ */
#endif

#ifdef DM9000A_UPTO_100M
	/* DM9000A�޴˼Ĵ��� */
	iow(DM9000_REG_OTCR, DM9000_OTCR_SET);			/* ����Ƶ�ʵ� 100Mhz ���� */
#endif

#ifdef  Dm9k_Int_Enable
	iow(DM9000_REG_IMR, DM9000_IMR_SET);			/* ���� �ж�ģʽ */
#else
	iow(DM9000_REG_IMR, DM9000_IMR_OFF);			/* �ر� �ж�ģʽ */
#endif
	iow(DM9000_REG_RCR, DM9000_RCR_SET);			/* ���� ���չ��� */

	TxPktCnt = 0;
	RxPktCnt = 0;
}

/*
 * ethernetif_output():
 *
 * This function is called by the TCP/IP stack when an IP packet
 * should be sent. It calls the function called low_level_output() to
 * do the actual transmission of the packet.
 *
 */

static err_t
ethernetif_output(struct netif *netif, struct pbuf *p,
      struct ip_addr *ipaddr)
{
//  struct ethernetif *ethernetif;
  /*
  struct pbuf *q;
  struct eth_hdr *ethhdr;
  struct eth_addr *dest, mcastaddr;
  struct ip_addr *queryaddr;
  err_t err;
  u8_t i;
  */
  
//  ethernetif = netif->state;

  /* resolve the link destination hardware address */
  p = etharp_output(netif, ipaddr, p);
  
  /* network hardware address obtained? */
  if (p == NULL)
  {
    /* we cannot tell if the packet was sent: the packet could */
    /* have been queued on an ARP entry that was already pending. */
  	return ERR_OK;
  }
  	
  /* send out the packet */
  return low_level_output(netif, p);

}

/*
 * ethernetif_input():
 *
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface.
 *
 */
void SystemReset(void);
static void
ethernetif_input(struct netif *netif)
{
	struct ethernetif *ethernetif;
	struct eth_hdr *ethhdr;
	struct pbuf *p, *q;
	//static uint8_t mac_offset;
	
	ethernetif = netif->state;
	
	while(1)
	{
		p = low_level_input(netif);
		
		if (p == NULL)
		return;
		
		#ifdef LINK_STATS
		lwip_stats.link.recv++;
		#endif /* LINK_STATS */
		
		ethhdr = p->payload;
		/*
		if(memcmp(&ethhdr->src,&netif->hwaddr,6)==0)
		{
			~*(unsigned char *)Flash_Read(&mac_offset,FLASH_DM9000_MAC+6,1);
			mac_offset++;
			mac_offset = ~mac_offset;
			Flash_Write(FLASH_DM9000_MAC+6,&mac_offset,1);
			SystemReset();
		}*/
		q = NULL;
		
		switch (htons(ethhdr->type))
		{
		case ETHTYPE_IP:
			q = etharp_ip_input(netif, p);
			pbuf_header(p, -14);
			netif->input(p, netif);
			break;
		case ETHTYPE_ARP:
			//q = etharp_arp_input(netif, ethernetif->ethaddr, p);
			q = etharp_arp_input(netif, ethernetif->ethaddr, p);
			break;
		default:
			pbuf_free(p);
			p = NULL;
			break;
		}
		if (q != NULL) {
			low_level_output(netif, q);
			pbuf_free(q);
			q = NULL;
		}
	}
}


/*******************************************************************************
*	������: low_level_init
*	��  ��: ��
*	��  ��: ��
*	��  ��: ��ʼ��DM9000AE
*/
void low_level_init(struct netif *netif)
{
	iow(DM9000_REG_NCR, DM9000_REG_RESET);			/* �� DM9000A ����������� */
	dm9k_udelay(10);								/* delay 10us */

	dm9k_reset();									/* ���� DM9000A ������� */
	dm9k_hash_table(netif);								/* ���� DM9000A MAC �� �ಥ*/

	iow(DM9000_REG_GPR, DM9000_PHY_OFF);			/* �ر� PHY ������ PHY ����*/
	dm9k_phy_write(0x00, 0x8000);					/* ���� PHY �ļĴ��� */
#ifdef DM9000A_FLOW_CONTROL
	dm9k_phy_write(0x04, 0x01e1 | 0x0400);			/* ���� ����Ӧģʽ���ݱ� */
#else
	dm9k_phy_write(0x04, 0x01e1);					/* ���� ����Ӧģʽ���ݱ� */
#endif
	//dm9k_phy_write(0x00, 0x1000);					/* ���� ��������ģʽ */
	/* ����ģʽ����
	  0x0000 : �̶�10M��˫��
	  0x0100 : �̶�10Mȫ˫��
	  0x2000 : �̶�100M��˫��
	  0x2100 : �̶�100Mȫ˫��
	  0x1000 : ����Ӧģʽ
	*/
	dm9k_phy_write(0x00, 0x1000);				/* ���� ��������ģʽ */

	iow(DM9000_REG_GPR, DM9000_PHY_ON);				/* ���� PHY ����, ���� PHY */

	//dm9k_debug_test();
}

static void
arp_timer(void *arg)
{
  etharp_tmr();
  sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
}

/*******************************************************************************
*	������: etherdev_init
*	��  ��: ��
*	��  ��: ��
*	��  ��: uIP �ӿں���,��ʼ������
*/
err_t dm9kif_init(struct netif *netif)
{
	struct ethernetif *ethernetif;
	ethernetif = mem_malloc(sizeof(struct ethernetif));
  	if (ethernetif == NULL) 
	{
		LWIP_DEBUGF(NETIF_DEBUG, ("dm9kif_init: out of memory\n")); 
		return ERR_MEM;
	}

	dm9000_tx_sem = OSSemCreate(1);
	netif->state = ethernetif;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;
	netif->output = ethernetif_output;
	netif->linkoutput = low_level_output;
	ethernetif->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);

	DM9K_CtrlLinesConfig();
	DM9K_FSMCConfig();

	s_FSMC_Init_Ok = 1;

	low_level_init(netif);
#ifdef Dm9k_Int_Enable
	DM9K_IntModeInit();
#endif

	etharp_init();
	sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
	return ERR_OK;
}


void DM9K_Dump(void)
{
	uint8_t save;
//	uint8_t i,j,k;
	save = NET_REG_ADDR;
	//printf("IMR:%02x\nISR:%02x\n",ior(DM9000_REG_IMR),ior(DM9000_REG_ISR));
	//printf("TSR1:%02x\nTSR2:%02x\n",ior(DM9000_REG_TSR1),ior(DM9000_REG_TSR2));
	//printf("RSR:%02x\nROCR:%02x\n",ior(DM9000_REG_RSR),ior(DM9000_REG_ROCR));
	//printf("Event:%d,%d\n",g_MaxEvent,g_CurEvent);
	//printf("PAR:%02x %02x %02x %02x %02x %02x\n",ior(DM9000_REG_PAR),ior(DM9000_REG_PAR+1),
	//ior(DM9000_REG_PAR+2),ior(DM9000_REG_PAR+3),
	//ior(DM9000_REG_PAR+4),ior(DM9000_REG_PAR+5));
	printf("NSR:%02x\n",ior(DM9000_REG_NSR));
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

