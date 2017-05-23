
/**************************************************************************************************
  Filename:       nwk_freq.c
  Revised:        $Date: 2008-03-19 13:44:02 -0700 (Wed, 19 Mar 2008) $
  Revision:       $Revision: 16613 $
  Author:         $Author: lfriedman $

  Description:    This file supports the SimpliciTI Freq network application.

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
#include "nwk.h"
#include "nwk_frame.h"
#include "nwk_freq.h"
#include "nwk_globals.h"
#include "nwk_api.h"

#if defined( FREQUENCY_AGILITY )
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
static freqEntry_t sCurLogicalChan;
static volatile uint8_t sTid = 0;

/******************************************************************************
 * LOCAL FUNCTIONS
 */

static fhStatus_t handle_freq_cmd(mrfiPacket_t *);
static fhStatus_t send_ping_reply(mrfiPacket_t *);
static void       change_channel_cmd(mrfiPacket_t *);

#ifdef ACCESS_POINT
static void    broadcast_channel_change(uint8_t);
#endif

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          nwk_freqInit
 *
 * @brief       Initialize NWK Frequency application.
 *
 * @return   none.
 */
void nwk_freqInit(void)
{

  memset(&sCurLogicalChan, 0x0, sizeof(sCurLogicalChan));

  /* pick a random value to start the transaction ID for this app. */
  sTid = MRFI_RandomByte();

  return;
}

/***************************************************************************
 * @fn          nwk_setChannel
 *
 * @brief       Set requested logical channel.  Accessed by application
 *              through IOCTL interface
 *
 * input parameters
 * @param   chan     - pointer to channel object of requested channel
 *
 * @return   status of operation:
 *             SMPL_SUCCESS    if channel set
 *             SMPL_NO_CHANNEL if channel not set
 */
smplStatus_t nwk_setChannel(freqEntry_t *chan)
{
  smplStatus_t rc = SMPL_NO_CHANNEL;

  if (chan->logicalChan < NWK_FREQ_TBL_SIZE)
  {
    MRFI_SetLogicalChannel(chan->logicalChan);
    sCurLogicalChan = *chan;
    rc = SMPL_SUCCESS;
  }
  return rc;
}

/******************************************************************************
 * @fn          nwk_getChannel
 *
 * @brief       Get current logical channel. Accessed by application through
 *              IOCTL interface
 *
 * input parameters
 * @param   chan     - pointer to channel object of requested channel
 *
 * output parameters
 * @param   chan     - populated channel object
 *
 * @return   none.
 */
void nwk_getChannel(freqEntry_t *chan)
{
  *chan = sCurLogicalChan;

  return;
}

/******************************************************************************
 * @fn          handle_freq_cmd
 *
 * @brief       Handle a Frequency application command.
 *
 * input parameters
 * @param   frame     - pointer to frame with command context
 *
 * @return   Return FHS_RELEASE if caller should release frame otherwise
 *           return FHS_REPLAY.
 */
static fhStatus_t handle_freq_cmd(mrfiPacket_t *frame)
{
  fhStatus_t rc = FHS_RELEASE;

  switch (*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS))
  {
    case FREQ_CMD_PING:
      rc = send_ping_reply(frame);
      break;

    case FREQ_CMD_MOVE:
      change_channel_cmd(frame);
      break;
#ifdef ACCESS_POINT
    case FREQ_CMD_REQ_MOVE:
      break;
#endif
    default:
      break;
  }

  return rc;
}

/********************************************************************************
 * @fn          change_channel_cmd
 *
 * @brief       Change to channel specified in received frame. Polling devices
 *              might be awake when the broadcast occurs but we want the channel
 *              change recovery to occur in a disciplined manner using the
 *              polling code. Also, with certain Test sceanrios in which a
 *              sleeping device is emulated we want to emulate 'missing' the
 *              broadcast change-channel command.
 *
 * input parameters
 * @param   frame     - pointer to frame containing target logical channel.
 *
 * @return   none.
 */
static void change_channel_cmd(mrfiPacket_t *frame)
{
#if !defined( RX_POLLS )
  freqEntry_t chan;

  chan.logicalChan = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+F_CHAN_OS);

  nwk_setChannel(&chan);
#endif
  return;
}

/******************************************************************************
 * @fn          send_ping_reply
 *
 * @brief       Send Frequency application ping reply.
 *
 * input parameters
 * @param   frame     - pointer to frame from pinger.
 *
 * @return   FHS_RELEASE unless this isn't an Access Point. In this case for
 *           flow to et this far it is a Range Extender, so replay the frame
 *           by returning FHW_REPLAY
 */
static fhStatus_t send_ping_reply(mrfiPacket_t *frame)
{
#ifdef ACCESS_POINT
  uint8_t      msg[FREQ_CMD_PING_FRAME_SIZE];
  frameInfo_t *pOutFrame;

  /* original request with reply bit on */
  msg[FB_APP_INFO_OS] = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS) | NWK_APP_REPLY_BIT;
  msg[FB_TID_OS]      = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+FB_TID_OS);

  if (pOutFrame = nwk_buildFrame(SMPL_PORT_FREQ, msg, sizeof(msg), MAX_HOPS_FROM_AP))
  {
    /* destination address is the source address of the received frame.*/
    memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);
    /* must use transaction ID of source frame*/
    PUT_INTO_FRAME(MRFI_P_PAYLOAD(&pOutFrame->mrfiPkt), F_TRACTID_OS, (GET_FROM_FRAME(MRFI_P_PAYLOAD(frame), F_TRACTID_OS)));
    nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED);
  }

  return FHS_RELEASE;
#else
  return FHS_REPLAY;
#endif  /* ACCESS_POINT */
}

/****************************************************************************************
 * @fn          nwk_processFreq
 *
 * @brief       Process a Frequency application frame.
 *
 * input parameters
 * @param   frame     - pointer to frame
 *
 * @return   Disposition for frame: either release (FHS_RELEASE) or replay (FHS_REPLAY).
 */
fhStatus_t nwk_processFreq(mrfiPacket_t *frame)
{
  fhStatus_t rc = FHS_RELEASE;
  uint8_t    replyType;

  /* Make sure this is a reply and see if we sent this. Validate the
   * packet for reception by client app.
   */
  if (SMPL_MY_REPLY == (replyType=nwk_isValidReply(frame, sTid, FB_APP_INFO_OS, FB_TID_OS)))
  {
    /* It's a match and it's a reply. Validate the received packet by
     * returning a 1 so it can be received by the client app.
     */
    rc = FHS_KEEP;
  }
#if !defined( END_DEVICE )
  else if (SMPL_A_REPLY == replyType)
  {
    /* no match. if i'm not an ED this is a reply that should be passed on. */
    rc = FHS_REPLAY;
  }
#endif  /* !END_DEVICE */
  else if (SMPL_NOT_REPLY == replyType)
  {
    rc = handle_freq_cmd(frame);
  }

  return rc;
}

/******************************************************************************
 * @fn          nwk_scanForChannels
 *
 * @brief       Scan for channels by sending a ping frame on each channel in the
 *              channel table and listen for a reply.
 *
 * input parameters
 * @param  channels    - pointer to area to receive list of channels from which
 *                       ping replies were received.
 *
 * output parameters
 * @param   channels   - populated list of channels.
 *
 * @return   statuis of operation..
 */
uint8_t nwk_scanForChannels(freqEntry_t *channels)
{
  uint8_t      spin;
  uint8_t      msg[FREQ_CMD_PING_FRAME_SIZE], i, num=0, notBcast = 1;
  addr_t      *apAddr, retAddr;
  uint8_t      radioState = MRFI_GetRadioState();
  freqEntry_t  chan;
  union
  {
    ioctlRawSend_t    send;
    ioctlRawReceive_t recv;
  } ioctl_info;

  /* send to AP. If we don't know AP address, broadcast. */
  apAddr = (addr_t *)nwk_getAPAddress();
  if (!apAddr)
  {
    apAddr = (addr_t *)nwk_getBCastAddress();
    notBcast = 0;
  }

  for (i=0; i<NWK_FREQ_TBL_SIZE; ++i)
  {
    chan.logicalChan = i;

    nwk_setChannel(&chan);

    ioctl_info.send.addr = apAddr;
    ioctl_info.send.msg  = msg;
    ioctl_info.send.len  = sizeof(msg);
    ioctl_info.send.port = SMPL_PORT_FREQ;

    msg[FB_APP_INFO_OS] = FREQ_CMD_PING;
    msg[FB_TID_OS]      = sTid;

    SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &ioctl_info.send);

    NWK_CHECK_FOR_SETRX();

    ioctl_info.recv.port = SMPL_PORT_FREQ;
    ioctl_info.recv.msg  = msg;
    ioctl_info.recv.addr = &retAddr;

    spin = NWK_RX_RETRY_COUNT;
    while (spin)
    {
    printf_1("==scanForChannels        wait ap \n");
      if (SMPL_SUCCESS == SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, &ioctl_info.recv))
      {
      /* Once we know the Access Point we're related to we only accept
       * ping replies from that one.
       */
        if (!notBcast || (notBcast && !memcmp(&retAddr, apAddr, NET_ADDR_SIZE)))
        {
          channels[num++].logicalChan = i;
          break;
        }
      }
      NWK_DELAY(WAIT_SHORT_TIME);
	  /*added by cbwang*/
	  NWK_DELAY(2);
	  /*end*/
      spin--;
    }

    NWK_CHECK_FOR_RESTORE_IDLE();

    sTid++;
    if (num && notBcast)
    {
      /* we're done...only one possible channel if we know the AP address. */
      break;
    }
    /* TODO: process encryption stuff */
  }

  return num;
}

/******************************************************************************
 * @fn          nwk_freqControl
 *
 * @brief       Handle application requests received through IOCTL interface.
 *
 * input parameters
 * @param   action  - requested action
 * @param   val     - pointer to parameters required/returned for action
 *
 * output parameters
 * @param   val   - populated values if action was a retrieval action.
 *
 * @return   status of operation.
 */
smplStatus_t nwk_freqControl(ioctlAction_t action, void *val)
{
  smplStatus_t rc;

  switch (action)
  {
    case IOCTL_ACT_SET:
#ifdef ACCESS_POINT
      broadcast_channel_change(((freqEntry_t *)val)->logicalChan);
#endif  /* ACCESS_POINT */
      rc = nwk_setChannel((freqEntry_t *)val);
      break;

    case IOCTL_ACT_GET:
      nwk_getChannel((freqEntry_t *)val);
      rc = SMPL_SUCCESS;
      break;

    case IOCTL_ACT_SCAN:
      {
        ioctlScanChan_t *sc = (ioctlScanChan_t *)val;

        sc->numChan = nwk_scanForChannels(sc->freq);
        rc = SMPL_SUCCESS;
      }
      break;

    default:
      rc = SMPL_NO_CHANNEL;
      break;
  }

  return rc;
}

/******************************************************************************
 * @fn          broadcast_channel_change
 *
* @brief       For Access Point only: broadcast a channel change frame.
 *
 * input parameters
 * @param   idx  -  index into channel table of new (logical) channel
 *
 * @return   none.
 */
#ifdef ACCESS_POINT
static void broadcast_channel_change(uint8_t idx)
{
  ioctlRawSend_t send;
  uint8_t        msg[FREQ_CMD_MOVE_FRAME_SIZE];

  if (idx >= NWK_FREQ_TBL_SIZE)
  {
    return;
  }

  msg[FB_APP_INFO_OS] = FREQ_CMD_MOVE;
  msg[F_CHAN_OS]      = idx;

  send.addr = (addr_t *)nwk_getBCastAddress();
  send.msg  = msg;
  send.len  = sizeof(msg);
  send.port = SMPL_PORT_FREQ;

  SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
}
#endif  /* ACCESS_POINT */

#else  /* FREQUENCY_AGILITY */

/**********************************************************************************
 * @fn          nwk_freqInit
 *
 * @brief       Initialize NWK Frequency application. Stub when Frequency Agility
 *              not supported.
 *
 * @return   none.
 */
void nwk_freqInit(void)
{
  return;
}

/****************************************************************************************
 * @fn          nwk_processFreq
 *
 * @brief       Process a Frequency application frame. Stub when Frequency Agility
 *              not supported.
 *
 * input parameters
 * @param   frame     - pointer to frame
 *
 * @return   Disposition for frame: either release (FHS_RELEASE) or replay (FHS_REPLAY).
 */
fhStatus_t nwk_processFreq(mrfiPacket_t *frame)
{
  return FHS_RELEASE;
}

#endif  /* FREQUENCY_AGILITY */
