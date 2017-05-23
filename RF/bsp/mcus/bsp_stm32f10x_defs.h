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

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS?
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
 *   MCU : Texas Instruments MSP430 family
 *   Microcontroller definition file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

#ifndef BSP_STM32F10X_DEFS_H
#define BSP_STM32F10X_DEFS_H

/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */
#define BSP_MCU_MSP430

/* ------------------------------------------------------------------------------------------------
 *                                     Compiler Abstraction
 * ------------------------------------------------------------------------------------------------
 */

/* ---------------------- IAR Compiler ---------------------- */
#ifdef __IAR_SYSTEMS_ICC__
#define BSP_COMPILER_IAR

#if (__VER__ < 342)
#error "ERROR: This IAR compiler port requires at least revision v3.42A."
/*
 *  Compiler versions previous to v3.42A do not have the universal msp430.h include file.
 *  To use an earlier version of the compiler, replace the above #error statement with
 *  the appropriate, device specific #include statement.
 */
#endif

/* workaround for release v3.42A - the msp430.h file did not include support for some devices */
#if ((__VER__ == 342) && (__SUBVERSION__ == 'A')) && \
    (defined (__MSP430F1610__) || defined (__MSP430F1611__) || defined (__MSP430F1612__))
#include <msp430x16x.h>
#else
#include <msp430.h>
#endif

#include <intrinsics.h>
#define __bsp_ENABLE_INTERRUPTS__()         __enable_interrupt()
#define __bsp_DISABLE_INTERRUPTS__()        __disable_interrupt()
#define __bsp_INTERRUPTS_ARE_ENABLED__()    (__get_SR_register() & GIE)

#define __bsp_ISTATE_T__                    istate_t
#define __bsp_GET_ISTATE__()                __get_interrupt_state()
#define __bsp_RESTORE_ISTATE__(x)           __set_interrupt_state(x)

#define __bsp_QUOTED_PRAGMA__(x)    _Pragma(#x)
#define __bsp_ISR_FUNCTION__(f,v)   __bsp_QUOTED_PRAGMA__(vector=v) __interrupt void f(void); \
                                    __bsp_QUOTED_PRAGMA__(vector=v) __interrupt void f(void)


/* ---------------------- Code Composer ---------------------- */
#elif (defined __TI_COMPILER_VERSION__) && (defined __MSP430__)
#define BSP_COMPILER_CODE_COMPOSER
#error "ERROR:  Support for Code Composer is not yet available."

#include <msp430xG46x.h> /* need generic support as in #include <msp430.h> */
#define __bsp_ENABLE_INTERRUPTS__()         _enable_interrupts()
#define __bsp_DISABLE_INTERRUPTS__()        _disable_interrupts()
#define __bsp_INTERRUPTS_ARE_ENABLED__()    (_get_SR_register() & GIE)

#define __bsp_ISTATE_T__                    unsigned short
#define __bsp_GET_ISTATE__()                (_get_SR_register() & GIE)
#define __bsp_RESTORE_ISTATE__(x)           _bis_SR_register(x)

#define __bsp_ISR_FUNCTION__(f,v)           void f(void); void f(void) /* stub, not implemented */


/* ------------------ Unrecognized Compiler ------------------ */
#else
/* changed by cbwang */
#include "hwinit.h"

//#error "ERROR: Unknown compiler."
#define __bsp_ENABLE_INTERRUPTS__()        irqEnableInterrupt() //__enable_interrupt()
#define __bsp_DISABLE_INTERRUPTS__()       irqDisableInterrupt() // __disable_interrupt()
#define __bsp_INTERRUPTS_ARE_ENABLED__()  ( g_pRegCpu->int0en & 0x01) //(__get_SR_register() & GIE)

#define __bsp_ISTATE_T__                    unsigned int //unsigned short
#define __bsp_GET_ISTATE__()               // __get_interrupt_state()
#define __bsp_RESTORE_ISTATE__(x)     //__set_interrupt_state(x)

#define __bsp_QUOTED_PRAGMA__(x)   // _Pragma(#x)
#define __bsp_ISR_FUNCTION__(f,v)   void f(void); void f(void) 


/* end */
#endif


/* ------------------------------------------------------------------------------------------------
 *                                          Common
 * ------------------------------------------------------------------------------------------------
 */
#define __bsp_LITTLE_ENDIAN__   1
#define __bsp_CODE_MEMSPACE__   /* blank */
#define __bsp_XDATA_MEMSPACE__  /* blank */


/**************************************************************************************************
 */
#endif