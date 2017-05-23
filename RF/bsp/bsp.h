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

/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Include file for core BSP services.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

#ifndef BSP_H
#define BSP_H

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdint.h>
#include "bsp_board_defs.h"
#include "bsp_macros.h"
#include "os_api.h"
#include "cc1100.h"
#include "printf_level.h"

/*added by cbwang@20080603*/
/* ------------------------------------------------------------------------------------------------
 *                                          Config Defines
 * ------------------------------------------------------------------------------------------------
  */
#define sio_printf(...)	printf(__VA_ARGS__)
//#define sio_printf(...)

#ifndef RF_SUCCESS
#define RF_SUCCESS	1
#endif
#ifndef RF_FAIL
#define RF_FAIL	0
#endif

#if 0
#define GPIO_RF_GDO0		7
#define GPIO_RF_GDO2		6
#define GPIO_RF_SO			3
#define GPIO_SPI_CSN		5
#define GPIO_SPI_SCLK		2
#define GPIO_SPI_SI			4
#define GPIO_SPI_SO			GPIO_RF_SO
#endif
//for cc1100 module 
#define _CC1100_RF_MODULE_
#define HGPIO_RF_TX_EN		2
#define HGPIO_RF_RX_EN		3
//

#ifndef WERS_AP_DEVICE
#ifdef WERS_EXTENDER_DEVICE

// Number of connections supported. each connection supports bi-directional
// communication.  Access Points and Range Extenders can set this to 0 if they
// do not host End Devices.
#define NUM_CONNECTIONS 0

// size of low level queues for sent and received frames. affects RAM usage

// AP needs larger input frame queue if it is supporting store-and-forward
// clients because the forwarded messages are held here. two is probably enough
// for an End Device
#define SIZE_INFRAME_Q  32

// the output frame queue can be small since Tx is done synchronously. if
// an Access Point device is also hosting an End Device that sends to a sleeping
// peer the output queue should be larger -- the waiting frames in this case
// are held here. in that case the output frame queue should be bigger. actually
// 1 is probably enough.
#define SIZE_OUTFRAME_Q 32

// this device's address. the first byte is used as a filter on the CC1100/CC2500
// radios so THE FIRST BYTE MUST NOT BE either 0x00 or 0xFF. also, for these radios
// on End Devices the first byte should be the least significant byte so the filtering
// is maximally effective. otherwise the frame has to be processed by the MCU before it
// is recognized as not intended for the device. APs and REs run in promiscuous mode so
// the filtering is not done. this macro intializes a static const array of unsigned
// characters of length NET_ADDR_SIZE (found in nwk_types.h). the quotes (") are
// necessary below unless the spaces are removed.
#define THIS_DEVICE_ADDRESS {0x77, 0x56, 0x34, 0x12}

// device type
#define RANGE_EXTENDER





// max hop count
#define MAX_HOPS 3

// max hops away from and AP. Keeps hop count and therefore replay
// storms down for sending to and from polling End Devices. Also used
// when joining since the EDs can't be more than 1 hop away.
#define MAX_HOPS_FROM_AP 1

// Maximum size of application payload
#define MAX_APP_PAYLOAD 50

// default Link token
#define DEFAULT_LINK_TOKEN 0x01020304

// default Join token
#define DEFAULT_JOIN_TOKEN 0x05060708

//#define FREQUENCY_AGILITY

#else
// Number of connections supported. each connection supports bi-directional
// communication.  Access Points and Range Extenders can set this to 0 if they
// do not host End Devices.
#define NUM_CONNECTIONS 2

// size of low level queues for sent and received frames. affects RAM usage

// AP needs larger input frame queue if it is supporting store-and-forward
// clients because the forwarded messages are held here. two is probably enough
// for an End Device
#define SIZE_INFRAME_Q 128
//#define SIZE_INFRAME_Q 16

// the output frame queue can be small since Tx is done synchronously. if
// an Access Point device is also hosting an End Device that sends to a sleeping
// peer the output queue should be larger -- the waiting frames in this case
// are held here. in that case the output frame queue should be bigger. actually
// 1 is probably enough.
#define SIZE_OUTFRAME_Q 32
//#define SIZE_OUTFRAME_Q 16

// this device's address. the first byte is used as a filter on the CC1100/CC2500
// radios so THE FIRST BYTE MUST NOT BE either 0x00 or 0xFF. also, for these radios
// on End Devices the first byte should be the least significant byte so the filtering
// is maximally effective. otherwise the frame has to be processed by the MCU before it
// is recognized as not intended for the device. APs and REs run in promiscuous mode so
// the filtering is not done. this macro intializes a static const array of unsigned
// characters of length NET_ADDR_SIZE (found in nwk_types.h). the quotes (") are
// necessary below unless the spaces are removed.
#define THIS_DEVICE_ADDRESS {0x73, 0x56, 0x34, 0x12}

// device type
#define END_DEVICE

// For End Devices we need to specify the Rx type. Uncomment the appropriate
// macro definition below. This field is used by the Access Point in networks
// containing an AP to know whether to reserve store-and-forward support for
// the joing End Device.
//
// For sleeping devices that do _not_ poll use RX_ALWAYS.
//
// For EDs that only send use RX_ALWAYS and not RX_NEVER. The RX_NEVER support
// is currently incomplete. It is inteneded to support devcies that can not send
// as opposed to devices that can but do not send.
//
//-DRX_LISTENS
//-DRX_POLLS
//-DRX_NEVER
#define RX_ALWAYS








// max hop count
#define MAX_HOPS 3

// max hops away from and AP. Keeps hop count and therefore replay
// storms down for sending to and from polling End Devices. Also used
// when joining since the EDs can't be more than 1 hop away.
#define MAX_HOPS_FROM_AP 1

// Maximum size of application payload
#define MAX_APP_PAYLOAD 50//10

// default Link token
#define DEFAULT_LINK_TOKEN 0x01020304

// default Join token
#define DEFAULT_JOIN_TOKEN 0x05060708

//#define FREQUENCY_AGILITY
/*added end*/
#endif
#else
// Number of connections supported. each connection supports bi-directional
// communication.  Access Points and Range Extenders can set this to 0 if they
// do not host End Devices.
#define NUM_CONNECTIONS 128// 8

// size of low level queues for sent and received frames. affects RAM usage

// AP needs larger input frame queue if it is supporting store-and-forward
// clients because the forwarded messages are held here.
#define SIZE_INFRAME_Q 128// 6

// the output frame queue can be small since Tx is done synchronously. if
// an Access Point device is also hosting an End Device that sends to a sleeping
// peer the output queue should be larger -- the waiting frames in this case
// are held here. in that case the output frame queue should be bigger.
#define SIZE_OUTFRAME_Q 32 //  2

// this device's address. the first byte is used as a filter on the CC1100/CC2500
// radios so THE FIRST BYTE MUST NOT BE either 0x00 or 0xFF. also, for these radios
// on End Devices the first byte should be the least significant byte so the filtering
// is maximally effective. otherwise the frame has to be processed by the MCU before it
// is recognized as not intended for the device. APs and REs run in promiscuous mode so
// the filtering is not done. this macro intializes a static const array of unsigned
// characters of length NET_ADDR_SIZE (found in nwk_types.h). the quotes (") are
// necessary below unless the spaces are removed.
#define THIS_DEVICE_ADDRESS {0xF1, 0x56, 0x34, 0x12}

// device type
#define ACCESS_POINT

// in the spcial case in which the AP is a data hub, the AP will automaically
// listen for a link each time a new device joins the network. this is a special
// case scenario in which all End Device peers are the AP and every ED links
// to the AP. in this scenario the ED must automatically try and link after the
// join reply.
#define AP_IS_DATA_HUB

// Store and forward support: number of clients
#define NUM_STORE_AND_FWD_CLIENTS 3



// max hop count
#define MAX_HOPS 3

// max hops away from and AP. Keeps hop count and therefore replay
// storms down for sending to and from polling End Devices. Also used
// when joining since the EDs can't be more than 1 hop away.
#define MAX_HOPS_FROM_AP 1

// Maximum size of application payload
#define MAX_APP_PAYLOAD 50//10

// default Link token
#define DEFAULT_LINK_TOKEN 0x01020304

// default Join token
#define DEFAULT_JOIN_TOKEN 0x05060708

//#define FREQUENCY_AGILITY

#endif

/* ------------------------------------------------------------------------------------------------
 *                                          BSP Defines
 * ------------------------------------------------------------------------------------------------
 */
#define BSP
#define BSP_VER       100  /* BSP version 1.00a */
#define BSP_SUBVER    a


/* ------------------------------------------------------------------------------------------------
 *                                            Clock
 * ------------------------------------------------------------------------------------------------
 */
#define BSP_CLOCK_MHZ   __bsp_CLOCK_MHZ__


/* ------------------------------------------------------------------------------------------------
 *                                            Memory
 * ------------------------------------------------------------------------------------------------
 */
#define BSP_LITTLE_ENDIAN   __bsp_LITTLE_ENDIAN__

#define CODE    __bsp_CODE_MEMSPACE__
#define XDATA   __bsp_XDATA_MEMSPACE__

/* ------------------------------------------------------------------------------------------------
 *                                            Interrupts
 * ------------------------------------------------------------------------------------------------
 */
/*
#define BSP_ISR_FUNCTION(func,vect)     __bsp_ISR_FUNCTION__(func,vect)

#define BSP_ENABLE_INTERRUPTS()         __bsp_ENABLE_INTERRUPTS__()
#define BSP_DISABLE_INTERRUPTS()        __bsp_DISABLE_INTERRUPTS__()
#define BSP_INTERRUPTS_ARE_ENABLED()    __bsp_INTERRUPTS_ARE_ENABLED__()
*/

/* ------------------------------------------------------------------------------------------------
 *                                         Critical Sections
 * ------------------------------------------------------------------------------------------------
 */
//typedef __bsp_ISTATE_T__  bspIState_t;
typedef OS_CPU_SR  bspIState_t;

#define BSP_ENTER_CRITICAL_SECTION(x)   ENTER_CRITICAL(x)	//st( x = __bsp_GET_ISTATE__(); __bsp_DISABLE_INTERRUPTS__(); )
#define BSP_EXIT_CRITICAL_SECTION(x)   EXIT_CRITICAL(x) 	//__bsp_RESTORE_ISTATE__(x)
#define BSP_CRITICAL_STATEMENT(x)       st( bspIState_t s;                    \
                                            BSP_ENTER_CRITICAL_SECTION(s);    \
                                            x;                                \
                                            BSP_EXIT_CRITICAL_SECTION(s); )


/* ------------------------------------------------------------------------------------------------
 *                                           Asserts
 * ------------------------------------------------------------------------------------------------
 */

/*
 *  BSP_ASSERT( expression ) - The given expression must evaluate as "true" or else the assert
 *  handler is called.  From here, the call stack feature of the debugger can pinpoint where
 * the problem occurred.
 *
 *  BSP_FORCE_ASSERT() - If asserts are in use, immediately calls the assert handler.
 *
 *  BSP_ASSERTS_ARE_ON - can use #ifdef to see if asserts are enabled
 *
 *  Asserts can be disabled for optimum performance and minimum code size (ideal for
 *  finalized, debugged production code).
 */
//#define BSP_NO_DEBUG
#if (!defined BSP_NO_DEBUG)
#ifndef BSP_ASSERT_HANDLER
//#define BSP_ASSERT_HANDLER()      st( sio_printf("ASSERT error!!--%s:%d\n", __FUNCTION__, __LINE__);__bsp_DISABLE_INTERRUPTS__();  while(1); )
#define BSP_ASSERT_HANDLER()      st( sio_printf("ASSERT error!!--%s:%d\n", __FUNCTION__, __LINE__);  while(1); )
#endif
#define BSP_ASSERT(expr)          st( if (!(expr)) BSP_ASSERT_HANDLER(); )
#define BSP_FORCE_ASSERT()        BSP_ASSERT_HANDLER()
#define BSP_ASSERTS_ARE_ON
#else
#define BSP_ASSERT(expr)          /* empty */
#define BSP_FORCE_ASSERT()        /* empty */
#endif

/* static assert */
#define BSP_STATIC_ASSERT(expr)   void bspDummyPrototype( char dummy[1/((expr)!=0)] );


/* ------------------------------------------------------------------------------------------------
 *                                           Prototypes
 * ------------------------------------------------------------------------------------------------
 */


void BSP_InitForCC1100(void);
void RegMrfiIsr(void (*pf)(void));
void ReleaseMrfiIsr(void);

#define RF_SUCCESS	1
#define RF_FAIL		0

/**************************************************************************************************
 */
#endif
