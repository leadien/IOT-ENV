/*
 * 10/20/2003: modefied for 51eda-arm-board          
 *              hgxxx <hgx2000@mail.china.com>
 * 04/27/2003: delete code: set mac addr manually, 
 *             add code: get mac addr from skyeye ne2k nic, which get from skyeye.conf
 *	 			chenyu <chenyu@hpclab.cs.tsinghua.edu.cn>
 * 01/21/2003: ne2k driver for lwip(OS independent) initial version
 * 				yangye <yangye@163.net>
 */


#include "lwip/debug.h"
#include "arch/cc.h"

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/ip.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"

#include "etharp.h"
#include "ne2kif.h"

#include "string.h"

#include "../../inc/44b.h"
#include "../../inc/44blib.h"

struct RTL8019if {
  struct eth_addr *ethaddr;
  /* Add whatever per-interface state that is needed here. */
};

const struct eth_addr ethbroadcast = {0xff,0xff,0xff,0xff,0xff,0xff};

struct netif *rtl8019if_netif;   //points to the real netif ,used by ne2k_isr

static void ne2k_copyin(u16_t count, u8_t *buf);
static void ne2k_copyout(u16_t count, u8_t *buf);
static void ne2k_discard(u16_t count);
void ne2k_isr(void);
static void low_level_init(struct netif * netif);
static struct pbuf * low_level_receive(struct RTL8019if *rtl8019if);
static err_t low_level_send(struct RTL8019if *rtl8019if,struct pbuf *p);



/*
 * Read the specified number of bytes from the device DMA port into
 * the supplied buffer.
 */
static void ne2k_copyin(u16_t len, u8_t *buf)
{
	u16_t count;
	u16_t *dataw;
	
	count = NE_WORDMODE ? len >> 1 : len;
	
  	#if NE_WORDMODE
    	dataw = (unsigned short *)buf;				// Use pointer for speed 
    	while(count--)                      		// Get words 
        	*dataw++ = NE_DATAW;
    	if (len & 0x01)                        		// If odd length, do last byte 
        	*(unsigned char *)dataw = NE_DATAB;
	#else
    	while(count--)                      		// Get bytes 
        	*buf++ = NE_DATAB;
	#endif
}

/*
 * Write the specified number of bytes from the device DMA port into
 * the supplied buffer.
 */ 
static void ne2k_copyout(u16_t len, u8_t *buf)
{
	u16_t count;
	
	count = NE_WORDMODE ? len >> 1 : len;
	
	#if NE_WORDMODE                   	// Word transfer?
		if((u32_t)buf & 0x01)
		{								// 若为奇数地址则按字节读取
	   		while(count--)
    		{
        		NE_DATAW = (*buf) | ((u16_t)*(buf + 1)) << 8;
				buf += 2;
    		}
        	if (len & 0x01)         	// If odd length, do last byte 
        		NE_DATAW = *buf;
		}
		else
		{								// 若为偶数地址则按半字读取
			u16_t *dataw;
			dataw = (u16_t *)buf;
			while(count--)
        		NE_DATAW = *dataw++;
        	if (len & 0x01)            	// If odd length, do last byte 
        		NE_DATAW = *(u8_t *)dataw;
		}
	#else
    	while(count--)       
        	NE_DATAB = *buf++;
	#endif	
}

static void ne2k_outpad(u16_t len)
{
	u16_t count;
	
	count = NE_WORDMODE ? len >> 1 : len;
	
	#if NE_WORDMODE                			/* Word transfer? */
    	while(count--)
        	NE_DATAW = 0;
	#else
    	while(count--)                  	/* O/P bytes */
        	NE_DATAB = 0;
	#endif	
}

/*
 * Pull the specified number of bytes from the device DMA port,
 * and throw them away.
 */
static void ne2k_discard(u16_t len)
{
	u8_t tmp;
	u16_t tmpw;
	u16_t count;
	
	count = NE_WORDMODE ? len >> 1 : len;
	
  	#if NE_WORDMODE
      	while(count--)                      		// Get words 
        	tmpw = NE_DATAW;
    	if (len & 0x01)                        		// If odd length, do last byte 
        	tmp = NE_DATAB;
	#else
    	while(count--)                      		// Get bytes 
        	tmp = NE_DATAB;
	#endif
}


// void NICISR(void) interrupt 
void ne2k_isr(void)
{
	u8_t  isr,curr,bnry;
	//	struct netif *netif;
	rI_ISPC = BIT_EINT3;
	rINTMSK = rINTMSK | BIT_EINT3;

	//close nic
	NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_STOP;
	
	//in PAGE0
	isr = NE_ISR;

	// ram overflow interrupt
	if (isr & ENISR_OVER)
	{
		NE_ISR = ENISR_OVER;				// clear interrupt
	}
	
	// error transfer interrupt ,NIC abort tx due to excessive collisions	
	if (isr & ENISR_TX_ERR)
	{
		NE_ISR = ENISR_TX_ERR;				// clear interrupt
	 	//temporarily do nothing
	}

	// Rx error , reset BNRY pointer to CURR (use SEND PACKET mode)
	if (isr & ENISR_RX_ERR)
	{
		NE_ISR = ENISR_RX_ERR;		// clear interrupt
		
		NE_CR = ENCR_PAGE1 | ENCR_NODMA | ENCR_STOP;
		curr = NE_CURR;
		NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_STOP;
		NE_BNRY = curr;
	}
	
	//got packet with no errors
	if (isr & ENISR_RX)
	{
		NE_ISR = ENISR_RX;		// clear interrupt

		NE_CR = ENCR_PAGE1 | ENCR_NODMA | ENCR_STOP;
		curr  = NE_CURR;
		NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_STOP;
		bnry = NE_BNRY;

		//get more than one packet until receive buffer is empty
		while(curr != bnry)
		{
			ne2k_recv_packet(rtl8019if_netif);
			NE_CR = ENCR_PAGE1 | ENCR_NODMA | ENCR_STOP;
			curr =  NE_CURR;
			NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_STOP;
			bnry = NE_BNRY;			
		}
	}
		
	//Transfer complelte, do nothing here
	if( isr & ENISR_TX)
	{
//		PRINT("ne2k_isr: is ISR_PTX\n");
		NE_ISR = ENISR_TX;          	// clear interrupt
	}
		
	NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_STOP;
	NE_ISR = 0xff;					// clear ISR	
	
	//open nic for next packet
	NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_START;

	rINTMSK = rINTMSK & (~BIT_EINT3);
	
}



/**
 * Initialize the rtk8019as, resetting the interface and getting the ethernet
 * address.
 */
static void 
low_level_init(struct netif * netif)
{
	u8_t temp;
	struct RTL8019if *rtl8019if;
	u8_t mac_addr[6];
	
//	NE_CR = 0x22;
	rtl8019if = netif->state;
	
	mac_addr[0] = 0;
	mac_addr[1] = 1;
	mac_addr[2] = 2;
	mac_addr[3] = 3;
	mac_addr[4] = 4;
	mac_addr[5] = 5;
	

	NE_CR = ENCR_PAGE0 + ENCR_NODMA + ENCR_START;
    Delay(100);	
	
	rPCONC |= 0x10000000;
	#if NE_WORDMODE
		rPDATC |= 0x4000;
	#else
		rPDATC &= 0xbfff;
	#endif
	Delay(500);
	temp = NE_RESET;
	NE_RESET = temp;
	Delay(500);
	rPDATC &= 0xbfff;
	rPCONC &= 0xcfffffff;

    NE_CR = ENCR_PAGE0 + ENCR_NODMA + ENCR_STOP;
    Delay(100);
    
    if(NE_ISR & ENISR_RESET)
    {
//    	Uart_Printf("\nNIC Reset OK!\n\n");
    }
    else
    {
    	Uart_Printf("\nNIC Reset Failed!\n\n");
    	return;
    }

    NE_DCR 		= NE_DCRVAL;
    NE_RBCR0 	= 0x00; 				/* MSB remote byte count reg */
    NE_RBCR1 	= 0x00; 				/* LSB remote byte count reg */
    NE_RCR 		= ENRCR_RXOFF; 			/* RX configuration reg    Monitor mode (no packet receive) */
    NE_TCR 		= ENTCR_TXOFF; 			/* TX configuration reg    set internal loopback mode  */
    NE_TPSR     = TX_START_PG;
    NE_PSTART   = RX_START_PG ; 		/* DMA START PAGE 46h */ 
    NE_PSTOP  	= RX_STOP_PG;    		/* Ending page +1 of ring buffer */
    NE_BNRY 	= RX_START_PG;			/* Boundary page of ring buffer */
    NE_ISR      = 0xff; 				/* INTerrupt stauts reg */
    NE_IMR      = ENIMR_RX | ENIMR_RX_ERR | ENIMR_TX | ENIMR_TX_ERR;
	
	//hgxxx 2003-10-11 add
    NE_CR = ENCR_PAGE1 + ENCR_NODMA + ENCR_STOP;
    Delay(100);
	
	NE_PAR0 = mac_addr[0];
	NE_PAR1 = mac_addr[1];
	NE_PAR2 = mac_addr[2];
	NE_PAR3 = mac_addr[3];
	NE_PAR4 = mac_addr[4];
	NE_PAR5 = mac_addr[5];
    NE_MAR0 	= 0xff;  
    NE_MAR1 	= 0xff;
    NE_MAR2 	= 0xff;
    NE_MAR3 	= 0xff;
    NE_MAR4 	= 0xff;
    NE_MAR5 	= 0xff;
    NE_MAR6 	= 0xff;
    NE_MAR7 	= 0xff;
    NE_CURR 	= RX_START_PG; 		/* RX_CURR_PG; Current memory page = RX_CURR_PG  ? */
  	/* make up an address. */
  	rtl8019if->ethaddr->addr[0] = mac_addr[0];
  	rtl8019if->ethaddr->addr[1] = mac_addr[1];
  	rtl8019if->ethaddr->addr[2] = mac_addr[2];
  	rtl8019if->ethaddr->addr[3] = mac_addr[3];
  	rtl8019if->ethaddr->addr[4] = mac_addr[4];
  	rtl8019if->ethaddr->addr[5] = mac_addr[5];
/*
	netif->hwaddr[0] = mac_addr[0];
	netif->hwaddr[1] = mac_addr[1];
	netif->hwaddr[2] = mac_addr[2];
	netif->hwaddr[3] = mac_addr[3];
	netif->hwaddr[4] = mac_addr[4];
	netif->hwaddr[5] = mac_addr[5];
*/	
    NE_CR = ENCR_PAGE0 + ENCR_NODMA ;  	/* 00001010B: PS1 PS0 RD2 RD1 RD0 TXP STA STP */
    Delay(100);
    NE_RCR 		= ENRCR_RXCONFIG;   	/* rx on(broadcasts, no multicast,errors   04*/
    NE_TCR 		= ENTCR_TXCONFIG; 		/* xmit on. */
    NE_ISR 		= 0xff; 	 			/* Individual bits are cleared by writing a "1" into it. */
    NE_IMR 		= ENISR_ALL; 			/* INTerrupt mask reg */
    
	NE_CR = ENCR_PAGE0 + ENCR_NODMA + ENCR_START;
	
 	rtl8019if_netif = netif;
}



/*
 * Function to do the actual writing of a packet into the devices
 * transmit buffer.  INT is disable during the function!!!!
 */
static err_t 
low_level_send(struct RTL8019if *rtl8019if,struct pbuf *p)
{
	struct pbuf *q;
	u8_t isr;
	u8_t chain;
	u8_t * tr_ptr;
	u16_t tr_len, temp_dw;
	u16_t padLength,packetLength;


	// Set up to transfer the packet contents to the NIC RAM.
	padLength = 0;
	packetLength = p->tot_len;
	// packetLength muse >=64 (see 802.3)
    if ((p->tot_len) < 64)
   {
       padLength = 64 - (p->tot_len);
       packetLength = 64;
   }
   	
 	//don't close nic,just close receive interrupt	
	NE_CR = ENCR_PAGE2 | ENCR_NODMA | ENCR_START;
	isr = NE_IMR;
	isr &= ~ENISR_RX;
    NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_START;
	NE_IMR = isr;
	
	NE_ISR = ENISR_RDC;		
	
    // Amount to send
	NE_RBCR0 = packetLength & 0xff;
	NE_RBCR1 = packetLength >> 8;

    // Address on NIC to store
	NE_RSAR0 = 0x00;
	NE_RSAR1 = NE_START_PG;
	// Write command to start
	NE_CR = ENCR_PAGE0 | ENCR_RWRITE | ENCR_START;    

	//write packet to ring buffers.
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
		
		ne2k_copyout(tr_len, tr_ptr);
			
		if (chain == 1) NE_DATAW = temp_dw;	
	}

	ne2k_outpad(padLength);
    
	// Wait for remote dma to complete - ISR Bit 6 clear if busy
	while((u8_t)(NE_ISR & ENISR_RDC) == 0 );
		
	NE_ISR = ENISR_RDC;     //clear RDC

	// Issue the transmit command.(start local dma)

 	NE_TPSR = NE_START_PG;
	NE_TBCR0 = packetLength & 0xff;
	NE_TBCR1 = packetLength >> 8;	
	// Start transmission (and shut off remote dma)
	NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_TRANS | ENCR_START;
	// reopen receive interrupt
	NE_CR = ENCR_PAGE2 | ENCR_NODMA | ENCR_START;
	isr = NE_IMR;
	isr |= ENISR_RX;
    NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_START;
	NE_IMR = isr;
	
	return ERR_OK;
}


/**
 * Read a packet into a pbuf chain.
 */
static struct pbuf * 
low_level_receive(struct RTL8019if *rtl8019if)
{
	u16_t  packetLength,len;
	u8_t PDHeader[18];   // Temp storage for ethernet headers
	struct pbuf * p;
	struct pbuf * q;
	u8_t * payload;
	

	NE_ISR = ENISR_RDC;
	NE_RBCR1 = 0x0f; 	// See controller manual , use send packet command
	NE_CR = ENCR_PAGE0 | ENCR_RREAD | ENCR_RWRITE | ENCR_START;

	// get the first 18 bytes from nic 
	ne2k_copyin(18,PDHeader);

	//  Store real length, set len to packet length - header
	packetLength = ((unsigned) PDHeader[2] | (PDHeader[3] << 8 ));

   	// verify if the packet is an IP packet or ARP packet
	if((PDHeader[3]>0x06)||(PDHeader[16] != 8)||(PDHeader[17] != 0 && PDHeader[17] != 6))
	{
		ne2k_discard(packetLength-14);
		return NULL;
	}  
    	
	// We allocate a pbuf chain of pbufs from the pool.
	p = pbuf_alloc(PBUF_LINK, packetLength, PBUF_POOL);
	if(p != NULL) 
	{
		// We iterate over the pbuf chain until we have read the entire
      	// packet into the pbuf.
		// This assumes a minimum pbuf size of 14 ... a good assumption
		memcpy(p->payload, PDHeader + 4, 14);
	
		for(q = p; q != NULL; q= q->next)
		{
     		// Read enough bytes to fill this pbuf in the chain. The
         	// avaliable data in the pbuf is given by the q->len variable.
      		// read data into(q->payload, q->len);
     		// First 14 bytes are already there, skip them
     		
			payload = q->payload;
			len = q->len;
			if (q == p)
			{
				payload += 14;
				len -=14;
			}
			ne2k_copyin(len,payload);
      		
		}
	} 
	else 
	{
    	// no more PBUF resource, Discard packet in buffer.
		ne2k_discard(packetLength-14);
  	}
	return p;
}


/*-----------------------------------------------------------------------------------*/
/*
 * ethernetif_init():
 *
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 *///  WARNING: must close all interrupts during init!!!!
/*-----------------------------------------------------------------------------------*/
void
ne2k_init(struct netif *netif)
{
	struct RTL8019if *rtl8019if;

	rtl8019if = mem_malloc(sizeof(struct RTL8019if));
	rtl8019if->ethaddr = mem_malloc(sizeof(struct eth_addr));
	netif->state = rtl8019if;
	netif->name[0] = 'e';
	netif->name[1] = 't';
	netif->output = ne2k_send_packet;
  
	low_level_init(netif);
	etharp_init();
}


/**
 * Send a packet to the RTK8019as from a series of pbuf buffers.
 */
err_t 
ne2k_send_packet(struct netif *netif, struct pbuf *p, struct ip_addr *ipaddr)
{
	struct RTL8019if *rtl8019if;
	struct pbuf *q;
	struct eth_hdr *ethhdr;
	struct eth_addr *dest, mcastaddr;
	struct ip_addr *queryaddr;
	u8_t i;
  
	rtl8019if = netif->state;

	// Make room for Ethernet header.
	if(pbuf_header(p, 14) != 0) 
	{
    	// The pbuf_header() call shouldn't fail, but we allocate an extra pbuf just in case.
    	q = pbuf_alloc(PBUF_LINK, 14, PBUF_RAM);
    	if(q == NULL) return ERR_MEM;
    	pbuf_chain(q, p);
    	p = q;
	}

  	// Construct Ethernet header. Start with looking up deciding which
    // MAC address to use as a destination address. Broadcasts and
    // multicasts are special, all other addresses are looked up in the
    // ARP table.
	queryaddr = ipaddr;
	if(ip_addr_isany(ipaddr) || ip_addr_isbroadcast(ipaddr, &(netif->netmask))) 
	{
    	dest = (struct eth_addr *)&ethbroadcast;
	} 
  	else if(ip_addr_ismulticast(ipaddr)) 
	{
    	// Hash IP multicast address to MAC address.
    	mcastaddr.addr[0] = 0x01;
    	mcastaddr.addr[1] = 0x0;
    	mcastaddr.addr[2] = 0x5e;
    	mcastaddr.addr[3] = ip4_addr2(ipaddr) & 0x7f;
    	mcastaddr.addr[4] = ip4_addr3(ipaddr);
    	mcastaddr.addr[5] = ip4_addr4(ipaddr);
    	dest = &mcastaddr;
	}
	else
	{
    	if(ip_addr_maskcmp(ipaddr, &(netif->ip_addr), &(netif->netmask))) 
		{
      		// Use destination IP address if the destination is on 
         	// the same subnet as we are.
      		queryaddr = ipaddr;
    	}
		else
		{
      		// Otherwise we use the default router as the address to send
         	// the Ethernet frame to. */
      		queryaddr = &(netif->gw);
    	}
    	dest = arp_lookup(queryaddr);
	}

	// If the arp_lookup() didn't find an address, we send out an ARP
	// query for the IP address.
	if(dest == NULL)
	{
    	q = etharp_query(netif, rtl8019if->ethaddr, queryaddr);
    	if(q != NULL)
		{
	  		low_level_send(rtl8019if, q);
      		pbuf_free(q);
      		return ERR_ARP;
    	}
    	return ERR_MEM;
	}
	ethhdr = p->payload;

  	for(i = 0; i < 6; i++)
	{
    	ethhdr->dest.addr[i] = dest->addr[i];
    	ethhdr->src.addr[i] = rtl8019if->ethaddr->addr[i];
	}
  
	ethhdr->type = htons(ETHTYPE_IP);
  
	return low_level_send(rtl8019if, p);
	
}


/**
 * Read a packet, clearing overflows.
 */
void 
ne2k_recv_packet(struct netif *netif)
{
	struct eth_hdr *ethhdr;
	struct RTL8019if *rtl8019if;
	struct pbuf *p;

	rtl8019if = netif->state;
// 	PRINT("start get a packet in ne2k_receive_packet\n ");
	p = low_level_receive(rtl8019if);
//	PRINT("got a packet from low_level_receive\n");
  
	if(p == NULL) return;
	
	ethhdr = p->payload;

	switch(htons(ethhdr->type))
	{
		case ETHTYPE_IP:
    		etharp_ip_input(netif, p);
			pbuf_header(p, -14);
			netif->input(p, netif);
			break;
		case ETHTYPE_ARP:
			p = etharp_arp_input(netif, rtl8019if->ethaddr, p);
    		if(p != NULL)
			{
	 			low_level_send(rtl8019if, p);
				pbuf_free(p);
    		}
    		break;
  		default:
    		pbuf_free(p);
    		break;
  	}
}

