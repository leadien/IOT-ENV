/**************************************************************************************************
  Filename:       nwk_ping.c
  Revised:        $Date: 2008-03-18 17:12:36 -0700 (Tue, 18 Mar 2008) $
  Revision:       $Revision: 16605 $
  Author:         $Author: lfriedman $

  Description:    This file supports the SimpliciTI Ping network application.

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

#define DEBUG_LEVEL 4
/******************************************************************************
 * INCLUDES
 */
#include <string.h>
#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_ping.h"
#include "nwk_globals.h"
#include "nwk_api.h"
#include "nwk_freq.h"


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

static volatile uint8_t sTid = 0;

/******************************************************************************
 * LOCAL FUNCTIONS
 */
static void smpl_send_ping_reply(mrfiPacket_t *);

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          nwk_pingInit
 *
 * @brief       Initialize Ping application.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   void
 */
void nwk_pingInit(void)
{
  sTid = MRFI_RandomByte();

  return;
}

/******************************************************************************
 * @fn          nwk_ping
 *
 * @brief       Called from the application level to ping a peer. A small
 *              payload is sent that includes a tid to detect correct reply.
 *              Caller does not supply payload.
 *
 * input parameters
 * @param   lid     - Link ID representing peer to ping
 *
 * output parameters
 *
 * @return   SMPL_SUCCESS   valid reply received
 *           SMPL_TIMEOUT   no valid reply received
 */
smplStatus_t nwk_ping(linkID_t lid)
{
  connInfo_t  *pCInfo = nwk_getConnInfo(lid);
  smplStatus_t rc = SMPL_BAD_PARAM;
  uint8_t      spin, done=0;
  uint8_t      msg[MAX_PING_APP_FRAME];
  uint8_t      radioState = MRFI_GetRadioState();
  union
  {
    ioctlRawSend_t    send;
    ioctlRawReceive_t recv;
  } ioctl_info;

  if (!pCInfo || (SMPL_LINKID_USER_UUD == lid))
  {
    /* either link ID bogus or tried to ping the unconnected user datagram link ID. */
    return rc;
  }

  {
#if defined( FREQUENCY_AGILITY )
    uint8_t  i, numChan;
    freqEntry_t channels[NWK_FREQ_TBL_SIZE];

    if (!(numChan=nwk_scanForChannels(channels)))
    {
      return SMPL_NO_CHANNEL;
    }

    for (i=0; i<numChan && !done; ++i)
    {
      nwk_setChannel(&channels[i]);
#else
    {
#endif

      ioctl_info.send.addr = (addr_t *)pCInfo->peerAddr;
      ioctl_info.send.msg  = msg;
      ioctl_info.send.len  = sizeof(msg);
      ioctl_info.send.port = SMPL_PORT_PING;

      /* fill in msg */
      msg[PB_APP_INFO_OS] = 1;
      msg[PB_TID_OS]      = sTid;

      SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &ioctl_info.send);

      NWK_CHECK_FOR_SETRX();

      ioctl_info.recv.port = SMPL_PORT_PING;
      ioctl_info.recv.msg  = msg;
      ioctl_info.recv.addr = 0;

      spin = NWK_RX_RETRY_COUNT;
      while (spin)
      {
      	NWK_DELAY(WAIT_FOR_REPLY);
	NWK_DELAY(WAIT_FOR_REPLY);
        if (SMPL_SUCCESS == SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, &ioctl_info.recv))
        {
          done = 1;
          sTid++;   /* guard against duplicates */
          break;
        }

        spin--;
      }

      NWK_CHECK_FOR_RESTORE_IDLE();

      // TODO: process encryption stuff
    }
  }

  return done ? SMPL_SUCCESS : SMPL_TIMEOUT;

}

static void smpl_send_ping_reply(mrfiPacket_t *frame)
{
  frameInfo_t *pOutFrame;

  /* Build the reply frame. The application payload is the one included in the
   * received frame payload.
   */
  if ( NULL != (pOutFrame = nwk_buildFrame(SMPL_PORT_PING, MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS, MRFI_GET_PAYLOAD_LEN(frame)-F_APP_PAYLOAD_OS, MAX_HOPS)))
  {
    /* destination address is the source adddress of the received frame. */
    memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);

    /* turn on the reply bit in the application payload */
    *(MRFI_P_PAYLOAD(&pOutFrame->mrfiPkt)+F_APP_PAYLOAD_OS+PB_APP_INFO_OS) |= NWK_APP_REPLY_BIT;
    nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED);
  }
}

fhStatus_t nwk_processPing(mrfiPacket_t *frame)
{
  fhStatus_t   rc = FHS_RELEASE;
  uint8_t      replyType;

  /* If we sent this then this is the reply. Validate the
   * packet for reception by client app. If we didn't send
   * it then we are the target. Send the reply.
   */
  if (SMPL_MY_REPLY == (replyType=nwk_isValidReply(frame, sTid, PB_APP_INFO_OS, PB_TID_OS)))
  {
    /* It's a match and it's a reply. Validate the received packet by
     * returning a 1 so it can be received by the client app.
     */
    rc = FHS_KEEP;
  }
#if !defined( END_DEVICE )
  else if (SMPL_A_REPLY == replyType)
  {
    /* no match. If I'm not an ED this is a reply that should be passed on. */
    rc = FHS_REPLAY;
  }
#endif  /* !END_DEVICE */
  else
  {
   #if 0 //orign version
    /* No, we didn't send it. Send reply assuming it's a Ping intended for us. */
    smpl_send_ping_reply(frame);
    rc = FHS_RELEASE;

#else
    /*modified by cbwang*/
#if defined( ACCESS_POINT)
#if 0
uint8_t i;
uint8_t *pdata = (uint8_t *)MRFI_P_SRC_ADDR(frame);
sio_printf("\n ping ");
for(i = 0; i < NET_ADDR_SIZE; i++)
{
	sio_printf("%02x",pdata[i]);
}
sio_printf("\n");
#endif

    if( nwk_isLinked(frame))
    {
    	smpl_send_ping_reply(frame);
    }
    else
    {
    	printf_0("\n\n@@not join\n\n");
    }
    /*end*/	
#elif defined(RANGE_EXTENDER)	
	printf_0("RX----\n");
	rc = FHS_REPLAY;	
#endif
#endif
  }

  (void) replyType;  // keep compiler happy when ED built...

  return rc;
}

