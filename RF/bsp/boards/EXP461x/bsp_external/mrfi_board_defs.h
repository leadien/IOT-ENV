/**************************************************************************************************
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Copyright 2007 Texas Instruments Incorporated.  All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Board definition file.
 *   Target : Texas Instruments MSP-EXP430FG4618
 *            "MSP430FG4618/F2013 Experimenter Board"
 *   Radios : CC2500, CC1100, CC1101
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

#ifndef MRFI_BOARD_DEFS_H
#define MRFI_BOARD_DEFS_H

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "bsp.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Defines
 * ------------------------------------------------------------------------------------------------
 */
#if (defined MRFI_CC2500)
  #define MRFI_BOARD_RSSI_OFFSET    72   /* no units */
#elif (defined MRFI_CC1100)
  #define MRFI_BOARD_RSSI_OFFSET    79   /* no units */
#elif (defined MRFI_CC1101)
  #define MRFI_BOARD_RSSI_OFFSET    74   /* no units */
#else
  #error "ERROR: RSSI offset value not defined for this radio"
#endif


/* Wait period in RX state before RSSI becomes valid.*/
/* Worst case time for RSSI to become valid is :
 * 757us - @ 2.4 kbps
 * 155us - @ 250 kbps
 * These numbers are from Design Note DN505 with added safety margin.
 */
 #if 1 //orign
#define MRFI_BOARD_RSSI_VALID_DELAY_US    1000
#else
#define MRFI_BOARD_RSSI_VALID_DELAY_US    1300
#endif

/* ------------------------------------------------------------------------------------------------
 *                                        Radio Selection
 * ------------------------------------------------------------------------------------------------
 */
#if (!defined MRFI_CC1100) && \
    (!defined MRFI_CC1101) && \
    (!defined MRFI_CC2500) && \
    (!defined MRFI_CC2420)
#error "ERROR: A compatible radio must be specified for the EXP461x board."
/*
 *  Since the EXP461x board can support several different radios, the installed
 *  radio must be specified with a #define.  It is best to do this at the
 *  project level.  However, if only one radio will ever be used, a #define
 *  could be placed here, above this error check.
 */
#endif

/*added by cbwang@20080603*/
//#include "hw/hw_disp.h"
//#include "hw/hw_cpu.h"
//#include "regmap/reg_hb.h"
#include "../../../../CC1100.h"

/*
extern genGpioReg_t *pgenGpioReg;

#define P1IN  pgenGpioReg->inVal[0] 
#define P1IE  pgenGpioReg->gpiooe
#define P1IFG_1 pgenGpioReg->fIrq[0]
#define P1IFG_2 pgenGpioReg->rIrq[0] 
#define P1IFES pgenGpioReg->fIrqEn[0]
#define P1IRES pgenGpioReg->rIrqEn[0]


#define P4DIR pgenGpioReg->gpiooe
#define P4OUT pgenGpioReg->gpioov
#define P4IN  pgenGpioReg->inVal[0] 

#define PIRQ pgenGpioReg->irq[0]
*/
/*added end*/


/* ------------------------------------------------------------------------------------------------
 *                                      GDO0 Pin Configuration
 * ------------------------------------------------------------------------------------------------
 */
#if 0
#define MRFI_TX_EN_PIN_AS_OUPUT()	 st( REG_HGPIO_OUTPUT_EN |= BV(__mrfi_TX_EN_BIT__); ) /* nothing required */
#define MRFI_RX_EN_PIN_AS_OUPUT()	st( REG_HGPIO_OUTPUT_EN  |= BV(__mrfi_RX_EN_BIT__); ) /* nothing required */

#define MRFI_TX_EN_HIGH()           st( REG_HGPIO_OUTPUT_VAL |=  BV(__mrfi_TX_EN_BIT__); )  /* atomic operation */
#define MRFI_TX_EN_LOW()            st( REG_HGPIO_OUTPUT_VAL &= ~BV(__mrfi_TX_EN_BIT__); ) /* atomic operation */
#define MRFI_RX_EN_HIGH()		st( REG_HGPIO_OUTPUT_VAL |=  BV(__mrfi_RX_EN_BIT__); )  /* atomic operation */
#define MRFI_RX_EN_LOW()		st( REG_HGPIO_OUTPUT_VAL &= ~BV(__mrfi_RX_EN_BIT__); ) /* atomic operation */
#else
//GPIO_TypeDef tmp;tmp->BSRR|MRFI_TX_EN_PIN
//#define MRFI_TX_EN_PIN_AS_OUPUT()	 (MRFI_TR_EN_PORT->BSRR=MRFI_TX_EN_PIN)
//#define MRFI_RX_EN_PIN_AS_OUPUT()	 (MRFI_TR_EN_PORT->BSRR=MRFI_RX_EN_PIN)

#define MRFI_TX_EN_HIGH()	 (MRFI_TR_EN_PORT->BSRR=MRFI_TX_EN_PIN)
#define MRFI_TX_EN_LOW()	 (MRFI_TR_EN_PORT->BRR=MRFI_TX_EN_PIN)
#define MRFI_RX_EN_HIGH()	 (MRFI_TR_EN_PORT->BSRR=MRFI_RX_EN_PIN)
#define MRFI_RX_EN_LOW()	 (MRFI_TR_EN_PORT->BRR=MRFI_RX_EN_PIN)
#endif

//#define MRFI_SYNC_PIN_IS_HIGH()                     MRFI_GDO0_PIN_IS_HIGH()
//#define MRFI_ENABLE_SYNC_PIN_INT()                  MRFI_ENABLE_GDO0_INT()
//#define MRFI_DISABLE_SYNC_PIN_INT()                 MRFI_DISABLE_GDO0_INT()
//#define MRFI_SYNC_PIN_INT_IS_ENABLED()              MRFI_GDO0_INT_IS_ENABLED()
//#define MRFI_CLEAR_SYNC_PIN_INT_FLAG()              MRFI_CLEAR_GDO0_INT_FLAG()
//#define MRFI_SYNC_PIN_INT_FLAG_IS_SET()             MRFI_GDO0_INT_FLAG_IS_SET()
//#define MRFI_CONFIG_SYNC_PIN_FALLING_EDGE_INT()     MRFI_CONFIG_GDO0_FALLING_EDGE_INT()
//GPIO_TypeDef tmp;
//EXTI_TypeDef tmpi;
#define MRFI_GDO0_PIN_IS_HIGH()				(MRFI_GDO_PORT->IDR&MRFI_GDO0_PIN)
//#define MRFI_ENABLE_GDO0_INT()
//#define MRFI_DISABLE_GDO0_INT()
//#define MRFI_ENABLE_GDO0_INT()		 		(EXTI->EMR |= MRFI_GDO0_PIN)
//#define MRFI_DISABLE_GDO0_INT()				(EXTI->EMR &= ~MRFI_GDO0_PIN)
#define MRFI_GDO0_INT_IS_ENABLED()			(EXTI->IMR & MRFI_GDO0_PIN)
#define MRFI_ENABLE_GDO0_INT()		 		NVIC_EnableIRQ(MRFI_GDO0_IRQ_CHANNEL)
#define MRFI_DISABLE_GDO0_INT()				NVIC_DisableIRQ(MRFI_GDO0_IRQ_CHANNEL)
//#define MRFI_GDO0_INT_IS_ENABLED()			NVIC_IRQisEnable(MRFI_GDO0_IRQ_CHANNEL)

#define MRFI_CLEAR_GDO0_INT_FLAG()			(EXTI->PR = MRFI_GDO0_LINE)
#define MRFI_GDO0_INT_FLAG_IS_SET()			(EXTI->PR & MRFI_GDO0_LINE)
//#define MRFI_CONFIG_GDO0_FALLING_EDGE_INT()	(tmpi->IDR|MRFI_PIN) 
#if 0
#define MRFI_SPI_DRIVE_CSN_HIGH()	SPI_CC1100_CS_HIGH()	/* atomic operation */
#define MRFI_SPI_DRIVE_CSN_LOW()	SPI_CC1100_CS_LOW()	/* atomic operation */
#define MRFI_SPI_CSN_IS_HIGH()		SPI_CC1100_CS_STAT()

#define MRFI_SPI_SO_IS_HIGH()		SPI_CC1100_SO_IS_HIGH()
#else
#define MRFI_SPI_DRIVE_CSN_HIGH()	(SPI_CC1100_CS_GPIO->BSRR=SPI_CC1100_CS)
#define MRFI_SPI_DRIVE_CSN_LOW()	(SPI_CC1100_CS_GPIO->BRR=SPI_CC1100_CS)
#define MRFI_SPI_CSN_IS_HIGH()		(SPI_CC1100_CS_GPIO->IDR&SPI_CC1100_CS) 

#define MRFI_SPI_SO_IS_HIGH()		(SPI_CC1100_GPIO->IDR&SPI_CC1100_PIN_MISO)
#endif


#if (defined MRFI_CC2500)
  #define MRFI_BOARD_RSSI_OFFSET    72   /* no units */
#elif (defined MRFI_CC1100)
  #define MRFI_BOARD_RSSI_OFFSET    79   /* no units */
#elif (defined MRFI_CC1101)
  #define MRFI_BOARD_RSSI_OFFSET    74   /* no units */
#else
  #error "ERROR: RSSI offset value not defined for this radio"
#endif

#define MRFI_SPI_INIT()	SPI_CC1100_Init()

/* read/write macros */
#define MRFI_SPI_WAIT_SO_IS_HIGH() 		while(MRFI_SPI_SO_IS_HIGH())
#define MRFI_SPI_WRITE_BYTE(x)			SPI_RxTxByte(x)			//  st(while (( P4IN & BV(__mrfi_SPI_SO_GPIO_BIT__) )); SPI_RxTxByte(x);)  //st( IFG2 &= ~URXIFG1;  U1TXBUF = x; )
#define MRFI_SPI_READ_BYTE()			SPI_RxTxByte(0)			//SPI_ReadByte()  //U1RXBUF
#define MRFI_SPI_WAIT_DONE()									//while(( P4IN & BV(__mrfi_SPI_SO_GPIO_BIT__))); //while(!(IFG2 & URXIFG1));
/* SPI critical section macros */
typedef bspIState_t mrfiSpiIState_t;
#define MRFI_SPI_ENTER_CRITICAL_SECTION(x)    BSP_ENTER_CRITICAL_SECTION(x)
#define MRFI_SPI_EXIT_CRITICAL_SECTION(x)     BSP_EXIT_CRITICAL_SECTION(x)

#define MRFI_SPI_IS_INITIALIZED()    1 //(U1TCTL & CKPH)


/* ------------------------------------------------------------------------------------------------
 *                                      GDO2 Pin Configuration
 * ------------------------------------------------------------------------------------------------
 */
#define __mrfi_GDO2_BIT__                     GPIO_RF_GDO2
#define MRFI_CONFIG_GDO2_PIN_AS_INPUT()      st( P1IE  &= ~BV(__mrfi_GDO2_BIT__); ) /* nothing required */
#define MRFI_GDO2_PIN_IS_HIGH()               (P1IN & BV(__mrfi_GDO2_BIT__))

#define MRFI_GDO2_INT_VECTOR                  PORT1_VECTOR
#define MRFI_ENABLE_GDO2_INT()                st(P1IFES |=  BV(__mrfi_GDO2_BIT__);)//st( P1IE  &= ~BV(__mrfi_GDO2_BIT__); ) /* atomic operation */
#define MRFI_DISABLE_GDO2_INT()               st(P1IFES &=  ~BV(__mrfi_GDO2_BIT__);)//st( P1IE  |=  BV(__mrfi_GDO2_BIT__); ) /* atomic operation */
#define MRFI_GDO2_INT_IS_ENABLED()            (  P1IFES  &   BV(__mrfi_GDO2_BIT__) )// (  P1IE  &   BV(__mrfi_GDO2_BIT__) )
#define MRFI_CLEAR_GDO2_INT_FLAG()            st( P1IFG &= ~BV(__mrfi_GDO2_BIT__); ) /* atomic operation */
#define MRFI_GDO2_INT_FLAG_IS_SET()            (  P1IFG &   BV(__mrfi_GDO2_BIT__) )
#define MRFI_CONFIG_GDO2_RISING_EDGE_INT()    st( P1IRES |= BV(__mrfi_GDO2_BIT__); ) /* atomic operation *///st( P1IES &= ~BV(__mrfi_GDO2_BIT__); ) /* atomic operation */
#define MRFI_CONFIG_GDO2_FALLING_EDGE_INT()  st( P1IFES |=  BV(__mrfi_GDO2_BIT__); )//st( P1IES |=  BV(__mrfi_GDO2_BIT__); ) /* atomic operation */


/* ------------------------------------------------------------------------------------------------
 *                                      SPI Configuration
 * ------------------------------------------------------------------------------------------------
 */
//  GPIO7--GDO0(input)    GPIO6--GDO2(input)    GPIO5--CSn(output)  
//  GPIO4--MOSI(output)   GPIO3--MISO(input)    GPIO2--SCLK(output)
//  input = 0,   output = 1

/* CSn Pin Configuration */
#if 0
#define __mrfi_SPI_CSN_GPIO_BIT__             GPIO_SPI_CSN
#define MRFI_SPI_CONFIG_CSN_PIN_AS_OUTPUT()   st( P4DIR |=  BV(__mrfi_SPI_CSN_GPIO_BIT__); )
#define MRFI_SPI_DRIVE_CSN_HIGH()             st( P4OUT |=  BV(__mrfi_SPI_CSN_GPIO_BIT__); ) /* atomic operation */
#define MRFI_SPI_DRIVE_CSN_LOW()              st( P4OUT &= ~BV(__mrfi_SPI_CSN_GPIO_BIT__); ) /* atomic operation */
#define MRFI_SPI_CSN_IS_HIGH()                   (  P4OUT &   BV(__mrfi_SPI_CSN_GPIO_BIT__) )
#endif

#if 0
/* SCLK Pin Configuration */
#define __mrfi_SPI_SCLK_GPIO_BIT__            GPIO_SPI_SCLK
#define MRFI_SPI_CONFIG_SCLK_PIN_AS_OUTPUT()  st( P4DIR |=  BV(__mrfi_SPI_SCLK_GPIO_BIT__); )
#define MRFI_SPI_DRIVE_SCLK_HIGH()            st( P4OUT |=  BV(__mrfi_SPI_SCLK_GPIO_BIT__); )
#define MRFI_SPI_DRIVE_SCLK_LOW()             st( P4OUT &= ~BV(__mrfi_SPI_SCLK_GPIO_BIT__); )

/* SI Pin Configuration */
#define __mrfi_SPI_SI_GPIO_BIT__              GPIO_SPI_SI
#define MRFI_SPI_CONFIG_SI_PIN_AS_OUTPUT()    st( P4DIR |=  BV(__mrfi_SPI_SI_GPIO_BIT__); )
#define MRFI_SPI_DRIVE_SI_HIGH()              st( P4OUT |=  BV(__mrfi_SPI_SI_GPIO_BIT__); )
#define MRFI_SPI_DRIVE_SI_LOW()               st( P4OUT &= ~BV(__mrfi_SPI_SI_GPIO_BIT__); )

/* SO Pin Configuration */
#define __mrfi_SPI_SO_GPIO_BIT__              GPIO_SPI_SO
#define MRFI_SPI_CONFIG_SO_PIN_AS_INPUT()     st( P4DIR  &= ~BV(__mrfi_SPI_SO_GPIO_BIT__); ) /* nothing to required */
#define MRFI_SPI_SO_IS_HIGH()                ( P4IN & BV(__mrfi_SPI_SO_GPIO_BIT__) )
#endif

#if 0//marked by cbwang
/* SPI Port Configuration */
#define MRFI_SPI_CONFIG_PORT()                st( P4SEL |= BV(__mrfi_SPI_SCLK_GPIO_BIT__) |  \
                                                           BV(__mrfi_SPI_SI_GPIO_BIT__)   |  \
                                                           BV(__mrfi_SPI_SO_GPIO_BIT__); )
#endif

/* read/write macros */
#if 0
#define MRFI_SPI_WAIT_SO_IS_HIGH() 	while (( P4IN & BV(__mrfi_SPI_SO_GPIO_BIT__) ));
#define MRFI_SPI_WRITE_BYTE(x)             SPI_RxTxByte(x);//  st(while (( P4IN & BV(__mrfi_SPI_SO_GPIO_BIT__) )); SPI_RxTxByte(x);)  //st( IFG2 &= ~URXIFG1;  U1TXBUF = x; )
#define MRFI_SPI_READ_BYTE()                 SPI_RxTxByte(0);//SPI_ReadByte()  //U1RXBUF
#define MRFI_SPI_WAIT_DONE()                 //while(( P4IN & BV(__mrfi_SPI_SO_GPIO_BIT__))); //while(!(IFG2 & URXIFG1));
#endif

/* SPI critical section macros */
typedef bspIState_t mrfiSpiIState_t;
#define MRFI_SPI_ENTER_CRITICAL_SECTION(x)    BSP_ENTER_CRITICAL_SECTION(x)
#define MRFI_SPI_EXIT_CRITICAL_SECTION(x)     BSP_EXIT_CRITICAL_SECTION(x)


/*
 *  Radio SPI Specifications
 * -----------------------------------------------
 *    Max SPI Clock   :  10 MHz
 *    Data Order      :  MSB transmitted first
 *    Clock Polarity  :  low when idle
 *    Clock Phase     :  sample leading edge
 */
/* initialization macro */
#if 0
#define MRFI_SPI_INIT() \
st ( \
  U1CTL  = SWRST;                     \
  U1CTL  = SWRST | MM | SYNC | CHAR;  \
  U1TCTL = CKPH | STC | SSEL1;        \
  U1BR0  = 2;                         \
  U1BR1  = 0;                         \
  U1MCTL = 0;                         \
  ME2 |= USPIE1;                      \
  MRFI_SPI_CONFIG_PORT();             \
  U1CTL &= ~SWRST;                    \
)
#else
#if 0
#define MRFI_SPI_INIT() \
st(\
genGpioReg_t *pgenGpioReg;\
pgenGpioReg = (genGpioReg_t *)0x10000000;\
pgenGpioReg->gpiooe  = (pgenGpioReg->gpiooe & 0xffffff37) | (pgenGpioReg->gpiooe | 0x34);\
)
#else
//#define MRFI_SPI_INIT()	SPI_Init()
#define MRFI_SPI_INIT()	SPI_CC1100_Init()
#endif
#endif
#define MRFI_SPI_IS_INITIALIZED()    1 //(U1TCTL & CKPH)


/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */
#ifndef BSP_BOARD_EXP461x
#error "ERROR: Mismatch between specified board and MRFI configuration."
#endif


/**************************************************************************************************
 */
#endif
