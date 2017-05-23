
/**************************************************************************************************
Filename:       nwk_globals.c
Revised:        $Date: 2008-04-11 15:41:27 -0700 (Fri, 11 Apr 2008) $
Revision:       $Revision: 16817 $
Author:         $Author: lfriedman $

Description:    This file manages global NWK data.

Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

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


/******************************************************************************
* INCLUDES
*/
#include <string.h>
#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_globals.h"

/******************************************************************************
* MACROS
*/

/******************************************************************************
* CONSTANTS AND DEFINES
*/

/******************************************************************************
* TYPEDEFS
*/

/******************************************************************************
* LOCAL VARIABLES
*/
static const addr_t sMyROMAddress = THIS_DEVICE_ADDRESS;
static addr_t       sAPAddress;
static addr_t       sMyRAMAddress;
static uint8_t      sRAMAddressIsSet = 0;
//added by cbwang
static addr_t sServerAddress = {0xFF, 0xFF, 0xFF, 0xFF };
//end
/******************************************************************************
* LOCAL FUNCTIONS
*/

/******************************************************************************
* GLOBAL VARIABLES
*/
/*changed by cbwang@20080603*/
//const __root uint8_t VersionString[] = "SimpliciTI V 1.0.5";
const  uint8_t VersionString[] = "SimpliciTI V 1.0.5";
volatile uint8_t g_RTS_process_flag = 1;
/*end*/

/******************************************************************************
* GLOBAL FUNCTIONS
*/

void nwk_globalsInit(void)
{
	if (!sRAMAddressIsSet)
	{
		memcpy(&sMyRAMAddress, &sMyROMAddress, sizeof(addr_t));
		sRAMAddressIsSet = 1;  /* RAM address is now valid */
	}

	return;
}

addr_t const *nwk_getMyAddress(void)
{
	/* This call supports returning a valid pointer before either the
	* initialization or external setting of the address, though the
	* value if this behavior isn't clear
	*/
	return sRAMAddressIsSet ? &sMyRAMAddress : &sMyROMAddress;
}

void nwk_setMyAddress(addr_t *addr)
{
	if (!sRAMAddressIsSet)
	{
		memcpy(&sMyRAMAddress, addr, sizeof(addr_t));
		sRAMAddressIsSet = 1;  /* RAM address is now valid */
	}

	return;
}

void nwk_setAPAddress(addr_t *addr)
{

	memcpy((void *)&sAPAddress, (void *)addr, NET_ADDR_SIZE);

	return;
}

addr_t const *nwk_getAPAddress(void)
{
	addr_t addr;

	memset(&addr, 0x0, sizeof(addr));

	return !memcmp(&sAPAddress, &addr, NET_ADDR_SIZE) ? 0 : &sAPAddress;
}

addr_t const *nwk_getBCastAddress(void)
{
	return (addr_t const *)mrfiBroadcastAddr;
}

//added by cbwang
void nwk_setServerAddress(addr_t *addr)
{
	if (addr){
		memcpy((void *)&sServerAddress, (void *)addr, NET_ADDR_SIZE);
	}

	return;
}

addr_t const *nwk_getServerAddress(void)
{
	return (addr_t const *)&sServerAddress;
}


void nwk_enableRTSProcess(void)
{
	g_RTS_process_flag = 1;
}


void nwk_disableRTSProcess(void)
{
	g_RTS_process_flag = 0;
}

//added end

