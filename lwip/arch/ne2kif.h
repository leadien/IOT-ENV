#ifndef __HGXXX_NE2K_H__
#define __HGXXX_NE2K_H__

#include "lwip/debug.h"
#include "cc.h"
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/ip.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "netif/etharp.h"

err_t ethernetif_init(struct netif *netif);

#define NE_WORDMODE		1

#define NE_START_PG		0x40     				// First page of TX buffer
#if	NE_WORDMODE
#define NE_STOP_PG		0x7e					// Last page + 1 of RX Ring
#define NE_DCRVAL		0x49
#else
#define NE_STOP_PG		0x5f
#define NE_DCRVAL		0x48
#endif

#define TX_PAGES		6       
#define	TX_START_PG		NE_START_PG
#define RX_START_PG		NE_START_PG + TX_PAGES
#define RX_STOP_PG		NE_STOP_PG
#define RX_CURR_PG		RX_START_PG + 1


#define NE_IO_BASE		0x06000300
#define NE_SHIFT(x)		(*(volatile unsigned char *)(NE_IO_BASE + ((x)<<1)))

#define NE_CR			NE_SHIFT(0x00)          //R/W，对不同的页，CR都是同一个
#define NE_DATAW		(*(volatile unsigned short *)(NE_IO_BASE + ((0x10)<<1)))
#define NE_DATAB		(*(volatile unsigned char  *)(NE_IO_BASE + ((0x10)<<1)))
#define	NE_RESET	   	NE_SHIFT(0x18)	     //0x18－0x1f 是网卡复位端口，重复，只用0x1f，读写它将复位网卡

#define NE_CLDA0		NE_SHIFT(0x01)			// Low byte of current local dma addr
#define NE_CLDA1		NE_SHIFT(0x02)			// High byte of current local dma addr
#define NE_TSR			NE_SHIFT(0x04)			// Transmit status reg
#define NE_NCR			NE_SHIFT(0x05)			// Number of collision reg
#define NE_FIFO			NE_SHIFT(0x06)			// FIFO
#define NE_CRDA0		NE_SHIFT(0x08)			// Low byte of current remote dma address
#define NE_CRDA1		NE_SHIFT(0x09)			// High byte of current remote dma address
#define NE_8019ID0		NE_SHIFT(0x0a)			// 
#define NE_8019ID1		NE_SHIFT(0x0b)			//
#define NE_RSR			NE_SHIFT(0x0c)			// Rx status reg
#define	NE_CNTR0		NE_SHIFT(0x0d)			// Rcv alignment error counter
#define NE_CNTR1		NE_SHIFT(0x0e)			// Rcv CRC error counter
#define NE_CNTR2		NE_SHIFT(0x0f)			// Rcv missed frame error counter

#define NE_PSTART		NE_SHIFT(0x01)			// Starting page of ring bfr
#define NE_PSTOP		NE_SHIFT(0x02)			// Ending page +1 of ring bfr
#define NE_TPSR			NE_SHIFT(0x04)			// Transmit starting page
#define NE_TBCR0		NE_SHIFT(0x05)			// Low byte of tx byte count
#define NE_TBCR1		NE_SHIFT(0x06)			// High byte of tx byte count
#define NE_RSAR0		NE_SHIFT(0x08)			// Low byte of remote start address reg
#define NE_RSAR1		NE_SHIFT(0x09)			// High byte of remote start address reg
#define NE_RBCR0		NE_SHIFT(0x0a)			// Low byte of remote byte count reg
#define NE_RBCR1		NE_SHIFT(0x0b)			// Highe byte of remote byte count reg
#define NE_RCR			NE_SHIFT(0x0c)			// RX configuration reg
#define NE_TCR			NE_SHIFT(0x0d)			// TX configuration reg
#define NE_DCR			NE_SHIFT(0x0e)			// Data configuration reg
#define NE_IMR			NE_SHIFT(0x0f)			// Interrupt mask reg

#define NE_BNRY			NE_SHIFT(0x03)			// Boundary page of ring bfr
#define NE_ISR			NE_SHIFT(0x07)			// Interrupt status reg

#define NE_PAR0			NE_SHIFT(0x01)			// This board's physical enet addr
#define NE_PAR1			NE_SHIFT(0x02)
#define NE_PAR2			NE_SHIFT(0x03)
#define NE_PAR3			NE_SHIFT(0x04)
#define NE_PAR4			NE_SHIFT(0x05)
#define NE_PAR5			NE_SHIFT(0x06)

#define NE_CURR			NE_SHIFT(0x07)			// Current memory page

#define NE_MAR0			NE_SHIFT(0x08)			// Multicast filter mask array
#define NE_MAR1			NE_SHIFT(0x09)
#define NE_MAR2			NE_SHIFT(0x0a)
#define NE_MAR3			NE_SHIFT(0x0b)
#define NE_MAR4			NE_SHIFT(0x0c)
#define NE_MAR5			NE_SHIFT(0x0d)
#define NE_MAR6			NE_SHIFT(0x0e)
#define NE_MAR7			NE_SHIFT(0x0f)

#define NE_9346CR		NE_SHIFT(0x01)
#define NE_BPAGE		NE_SHIFT(0x02)
#define NE_CONFIG0		NE_SHIFT(0x03)
#define NE_CONFIG1		NE_SHIFT(0x04)
#define NE_CONFIG2		NE_SHIFT(0x05)
#define NE_CONFIG3		NE_SHIFT(0x06)
#define NE_TEST			NE_SHIFT(0x07)
#define NE_CSNSAV		NE_SHIFT(0x08)
#define NE_HLTCLK		NE_SHIFT(0x09)

#define NE_INTR			NE_SHIFT(0x0b)
#define NE_FMWP			NE_SHIFT(0x0c)
#define NE_CONFIG4		NE_SHIFT(0x0d)

#define ENCR_STOP	    0x01					// Stop and reset the chip         
#define ENCR_START	    0x02					// Start the chip, clear reset 
#define ENCR_TRANS	    0x04					// Transmit a frame        
#define ENCR_RREAD	    0x08					// Remote read             
#define ENCR_RWRITE	    0x10					// Remote write       
#define ENCR_NODMA	    0x20					// Remote DMA              
#define ENCR_PAGE0	    0x00					// Select page chip registers        
#define ENCR_PAGE1	    0x40					// using the two high-order bits   
#define ENCR_PAGE2	    0x80 
#define ENCR_PAGE3	    0xc0 

#define	ENISR_RX 		0x01					// Receiver, no error   
#define	ENISR_TX	    0x02					// Transceiver, no error  
#define	ENISR_RX_ERR	0x04					// Receiver, with error    
#define	ENISR_TX_ERR	0x08					// Transmitter, with error     
#define	ENISR_OVER	    0x10					// Receiver overwrote the ring    
#define	ENISR_COUNTERS	0x20					// Counters need emptying    
#define	ENISR_RDC	    0x40					// remote dma complete 
#define	ENISR_RESET     0x80					// Reset completed      
#define	ENISR_ALL	    0x35					// 3f  Interrupts we will enable 	

#define ENIMR_RX		0x01
#define ENIMR_TX		0x02
#define ENIMR_RX_ERR	0x04
#define ENIMR_TX_ERR	0x08
#define ENIMR_OVER		0x10
#define ENIMR_COUNTERS	0x20
#define ENIMR_RDC		0x40

#define ENTSR_PTX	    0x01					// Packet transmitted without error 
#define ENTSR_ND	    0x02					// The transmit wasn't deferred
#define ENTSR_COL	    0x04					// The transmit collided at least once
#define ENTSR_ABT	    0x08					// The transmit collided 16 times, and was deferred
#define ENTSR_CRS	    0x10					// The carrier sense was lost
#define ENTSR_FU	    0x20					// A "FIFO underrun" occurred during transmit
#define ENTSR_CDH	    0x40					// The collision detect "heartbeat" signal was lost
#define ENTSR_OWC	    0x80					// There was an out-of-window collision

#define ENRSR_RXOK  	0x01					// Received a good packet 	
#define ENRSR_CRC	    0x02					// CRC error(CNTR1++) 
#define ENRSR_FAE	    0x04					// Frame alignment error(CNTR0++)
#define ENRSR_FO	    0x08					// FIFO overrun when receiving 
#define ENRSR_MPA	    0x10					// Missed packet.. Gap area isn't exist 
#define ENRSR_PHY	    0x20					// Physical/multicase address 
#define ENRSR_DIS	    0x40					// Receiver disable.set in monitor mode 
#define ENRSR_DEF	    0x80					// Deferring because of collision  

#define ENTCR_TXCONFIG 	0x00    				// Normal transmit mode 
#define ENTCR_CRC	    0x01					// inhibit CRC,do not append crc when 1 
#define ENTCR_LOOP	    0x02					// set internal loopback mode 
#define ENTCR_LB01	    0x06					// encoded loopback control 
#define ENTCR_ATD	    0x08					// auto tx disable  
#define ENTCR_OFST	    0x10					// collision offset enable
#define ENTCR_TXOFF		0x02

#define ENRCR_RXCONFIG 	0x04 					// EN0_RXCR: broadcasts,no multicast,errors
#define ENRCR_CRC	    0x01					// Save error packets(admit) 
#define ENRCR_RUNT	    0x02					// Accept runt pckt(below 64bytes) 
#define ENRCR_BCST	    0x04					// Accept broadcasts when 1 
#define ENRCR_MULTI		0x08					// Multicast (if pass filter) when 0 
#define ENRCR_PROMP		0x10					// Promiscuous physical addresses when 1
#define ENRCR_MON	    0x20					// Monitor mode (no packets rcvd)
#define ENRCR_RXOFF		0x20 

#define ENDCR_WTS	    0x01					// byte transfer mode selection 0/1 byte/word width 
#define ENDCR_BOS	    0x02					// byte order selection              
#define ENDCR_LAS	    0x04					// long addr selection (must be set to zero) 
#define ENDCR_BMS	    0x08					// loopback select       set is normal  mode    
#define ENDCR_ARM	    0x10					// autoinitialize remote when 1     
#define ENDCR_FT00	    0x00					// fifo treshold   2 byte          
#define ENDCR_FT01	    0x20					//                  4 byte    
#define ENDCR_FT10	    0x40					//                  8 byte     
#define ENDCR_FT11	    0x60					//                 12 byte      

#ifndef PRINT
#define		PRINT	printf
#endif  /* PRINT */

#define NE2KIF_DEBUG 1

#endif
