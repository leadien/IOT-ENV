/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

/*
 * modified by hugang <hgxxx@51eda.com>
 * date : 2004-07-02
 */

 
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/stats.h"
#include "lwip/err.h"
#include "lwip/debug.h"

#include "netif/etharp.h"

#include "ne2kif.h"
#include "string.h"
#include "inc/44b.h"
#include "inc/44blib.h"

/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'n'

struct ethernetif {
  struct eth_addr *ethaddr;
  /* Add whatever per-interface state that is needed here. */
};

static const struct eth_addr ethbroadcast = {{0xff,0xff,0xff,0xff,0xff,0xff}};
extern struct netif *rtl8019_netif;

static void ethernetif_input(struct netif *netif);
static err_t ethernetif_output(struct netif *netif, struct pbuf *p, struct ip_addr *ipaddr);

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
  {								/* 若为奇数地址则按字节读取*/
    while(count--) {
      NE_DATAW = (*buf) | ((u16_t)*(buf + 1)) << 8;
      buf += 2;
    }
    if (len & 0x01)         	/* If odd length, do last byte */
      NE_DATAW = *buf;
  }
  else
  {								/* 若为偶数地址则按半字读取 */
    u16_t *dataw;
    dataw = (u16_t *)buf;
    while(count--)
      NE_DATAW = *dataw++;
    if (len & 0x01)            	/* If odd length, do last byte */
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
  while(count--)                      		
    tmpw = NE_DATAW;
  if (len & 0x01)                        		
    tmp = NE_DATAB;
#else
  while(count--)                      	
    tmp = NE_DATAB;
#endif
}

/* void NICISR(void) interrupt */
void ne2k_isr(void)
{
  u8_t  isr,curr,bnry;

  rI_ISPC = BIT_EINT3;
  rINTMSK = rINTMSK | BIT_EINT3;

  /* close nic */
  NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_STOP;
  isr = NE_ISR;

  if (isr & ENISR_OVER) {
    NE_ISR = ENISR_OVER;				
  }
	
  if (isr & ENISR_TX_ERR) {
    NE_ISR = ENISR_TX_ERR;				
  }

  /* Rx error , reset BNRY pointer to CURR (use SEND PACKET mode) */
  if (isr & ENISR_RX_ERR) {
    NE_ISR = ENISR_RX_ERR;
		
    NE_CR = ENCR_PAGE1 | ENCR_NODMA | ENCR_STOP;
    curr = NE_CURR;
    NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_STOP;
    NE_BNRY = curr;
  }
	
  /* got packet with no errors */
  if (isr & ENISR_RX) {
    NE_ISR = ENISR_RX;

    NE_CR = ENCR_PAGE1 | ENCR_NODMA | ENCR_STOP;
    curr  = NE_CURR;
    NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_STOP;
    bnry = NE_BNRY;

    /* get more than one packet until receive buffer is empty */
    while(curr != bnry) {
	  ethernetif_input(rtl8019_netif);
      NE_CR = ENCR_PAGE1 | ENCR_NODMA | ENCR_STOP;
      curr =  NE_CURR;
      NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_STOP;
      bnry = NE_BNRY;			
    }
  }
		
  /* Transfer complelte, do nothing here */
  if( isr & ENISR_TX) {
    NE_ISR = ENISR_TX;
  }
		
  NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_STOP;
  NE_ISR = 0xff;
	
  /* open nic for next packet */
  NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_START;

  rINTMSK = rINTMSK & (~BIT_EINT3);
	
}


static void
low_level_init(struct netif *netif)
{
  struct ethernetif *ethernetif;
  u8_t mac_addr[6];
  u8_t temp;
	
  mac_addr[0] = 0;
  mac_addr[1] = 1;
  mac_addr[2] = 2;
  mac_addr[3] = 3;
  mac_addr[4] = 4;
  mac_addr[5] = 5;

  ethernetif = netif->state;
  
  /* set MAC hardware address length */
  netif->hwaddr_len = 6;

  /* set MAC hardware address */
  netif->hwaddr[0] = mac_addr[0];
  netif->hwaddr[1] = mac_addr[1];
  netif->hwaddr[2] = mac_addr[2];
  netif->hwaddr[3] = mac_addr[3];
  netif->hwaddr[4] = mac_addr[4];                
  netif->hwaddr[5] = mac_addr[5];
                     
  /* maximum transfer unit */
  netif->mtu = 1500; 
  
  /* broadcast capability */
  netif->flags = NETIF_FLAG_BROADCAST;
 
  /* Do whatever else is needed to initialize interface. */  
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
  
  NE_DCR 	= NE_DCRVAL;
  NE_RBCR0 	= 0x00; 				/* MSB remote byte count reg */
  NE_RBCR1 	= 0x00; 				/* LSB remote byte count reg */
  NE_RCR 	= ENRCR_RXOFF; 			/* RX configuration reg    Monitor mode (no packet receive) */
  NE_TCR 	= ENTCR_TXOFF; 			/* TX configuration reg    set internal loopback mode  */
  NE_TPSR   = TX_START_PG;
  NE_PSTART = RX_START_PG ; 		/* DMA START PAGE 46h */ 
  NE_PSTOP  = RX_STOP_PG;    		/* Ending page +1 of ring buffer */
  NE_BNRY 	= RX_START_PG;			/* Boundary page of ring buffer */
  NE_ISR    = 0xff; 				/* INTerrupt stauts reg */
  NE_IMR    = ENIMR_RX | ENIMR_RX_ERR | ENIMR_TX | ENIMR_TX_ERR;
	
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
  
  NE_CR = ENCR_PAGE0 + ENCR_NODMA ;  	/* 00001010B: PS1 PS0 RD2 RD1 RD0 TXP STA STP */
  Delay(100);
  NE_RCR 		= ENRCR_RXCONFIG;   	/* rx on(broadcasts, no multicast,errors   04*/
  NE_TCR 		= ENTCR_TXCONFIG; 		/* xmit on. */
  NE_ISR 		= 0xff; 	 			/* Individual bits are cleared by writing a "1" into it. */
  NE_IMR 		= ENISR_ALL; 			/* INTerrupt mask reg */
    
  NE_CR = ENCR_PAGE0 + ENCR_NODMA + ENCR_START;  
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

  u8_t isr;
  u8_t chain;
  u8_t * tr_ptr;
  u16_t tr_len, temp_dw;
  u16_t padLength,packetLength;

  /* Set up to transfer the packet contents to the NIC RAM. */
  padLength = 0;
  packetLength = p->tot_len;
  
  /* packetLength muse >=64 (see 802.3) */
  if ((p->tot_len) < 64)
  {
    padLength = 64 - (p->tot_len);
    packetLength = 64;
  }
   	
  /* don't close nic,just close receive interrupt */
  NE_CR = ENCR_PAGE2 | ENCR_NODMA | ENCR_START;
  isr = NE_IMR;
  isr &= ~ENISR_RX;
  NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_START;
  NE_IMR = isr;
	
  NE_ISR = ENISR_RDC;		
	
  /* Amount to send */
  NE_RBCR0 = packetLength & 0xff;
  NE_RBCR1 = packetLength >> 8;

  /* Address on NIC to store */
  NE_RSAR0 = 0x00;
  NE_RSAR1 = NE_START_PG;
  
  /* Write command to start */
  NE_CR = ENCR_PAGE0 | ENCR_RWRITE | ENCR_START;

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
		
	ne2k_copyout(tr_len, tr_ptr);
			
	if (chain == 1) NE_DATAW = temp_dw;	
  }

  ne2k_outpad(padLength);
    
  /* Wait for remote dma to complete - ISR Bit 6 clear if busy */
  while((u8_t)(NE_ISR & ENISR_RDC) == 0 );

  /* clear RDC */ 		
  NE_ISR = ENISR_RDC;     

  /* Issue the transmit command.(start local dma) */
  NE_TPSR = NE_START_PG;
  NE_TBCR0 = packetLength & 0xff;
  NE_TBCR1 = packetLength >> 8;	
  
  /* Start transmission (and shut off remote dma) */
  NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_TRANS | ENCR_START;
  /* reopen receive interrupt */
  NE_CR = ENCR_PAGE2 | ENCR_NODMA | ENCR_START;
  isr = NE_IMR;
  isr |= ENISR_RX;
  NE_CR = ENCR_PAGE0 | ENCR_NODMA | ENCR_START;
  NE_IMR = isr;
  
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

static struct pbuf *
low_level_input(struct netif *netif)
{
  struct pbuf *p, *q;
  u16_t packetLength, len;
  u8_t PDHeader[18];   /* Temp storage for ethernet headers */
  u8_t * payload;

  NE_ISR = ENISR_RDC;
  NE_RBCR1 = 0x0f; 	/* See controller manual , use send packet command */
  NE_CR = ENCR_PAGE0 | ENCR_RREAD | ENCR_RWRITE | ENCR_START;

  /* get the first 18 bytes from nic */
  ne2k_copyin(18,PDHeader);

  /* Store real length, set len to packet length - header */
  packetLength = ((unsigned) PDHeader[2] | (PDHeader[3] << 8 ));

  /* verify if the packet is an IP packet or ARP packet */
  if((PDHeader[3]>0x06)||(PDHeader[16] != 8)||(PDHeader[17] != 0 && PDHeader[17] != 6))
  {
	ne2k_discard(packetLength-14);
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
	  
      ne2k_copyin(len,payload);
    }

#ifdef LINK_STATS
    lwip_stats.link.recv++;
#endif /* LINK_STATS */      
  } else {
    /* no more PBUF resource, Discard packet in buffer. */  	
    ne2k_discard(packetLength-14);
#ifdef LINK_STATS
    lwip_stats.link.memerr++;
    lwip_stats.link.drop++;
#endif /* LINK_STATS */      
  }

  return p;  
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
  struct ethernetif *ethernetif;
  /*
  struct pbuf *q;
  struct eth_hdr *ethhdr;
  struct eth_addr *dest, mcastaddr;
  struct ip_addr *queryaddr;
  err_t err;
  u8_t i;
  */
  
  ethernetif = netif->state;

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

static void
ethernetif_input(struct netif *netif)
{
  struct ethernetif *ethernetif;
  struct eth_hdr *ethhdr;
  struct pbuf *p, *q;

  ethernetif = netif->state;
  
  p = low_level_input(netif);

  if (p == NULL)
    return;

#ifdef LINK_STATS
  lwip_stats.link.recv++;
#endif /* LINK_STATS */

  ethhdr = p->payload;
  q = NULL;
    
  switch (htons(ethhdr->type)) {
    case ETHTYPE_IP:
      q = etharp_ip_input(netif, p);
      pbuf_header(p, -14);
      netif->input(p, netif);
      break;
      
    case ETHTYPE_ARP:
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

static void
arp_timer(void *arg)
{
  etharp_tmr();
  sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
}

/*
 * ethernetif_init():
 *
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 */

err_t
ethernetif_init(struct netif *netif)
{
  struct ethernetif *ethernetif;
    
  ethernetif = mem_malloc(sizeof(struct ethernetif));
  
  if (ethernetif == NULL)
  {
  	LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_init: out of memory\n"));
  	return ERR_MEM;
  }
  
  netif->state = ethernetif;
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  netif->output = ethernetif_output;
  netif->linkoutput = low_level_output;
  
  ethernetif->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);
  
  low_level_init(netif);

  etharp_init();

  sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);

  return ERR_OK;
}

