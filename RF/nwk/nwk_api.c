
/**************************************************************************************************
  Filename:       nwk_api.c
  Revised:        $Date: 2008-04-09 12:01:25 -0700 (Wed, 09 Apr 2008) $
  Revision:       $Revision: 16783 $
  Author:         $Author: lfriedman $

  Description:    This file supports the SimpliciTI appliction layer API.

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
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_app.h"
#include "mrfi.h"
#include "nwk_globals.h"
#include "nwk_freq.h"

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS AND DEFINES
 */

/* These defines are in support an application listening for a link frame to
 * terminate after some amount of time. The intention is that this guard be
 * the exception. The intention of the SimpliciTI design is that the
 * temporal contiguity between the listen and the reception of the link frame
 * from the peer be very tight. The SMPL_LinkListen() should be termninated
 * by the reception of the link frame. But in case it does not receive the frame
 * the support below allows intervention by the application.
 */

/* The intention is for user to modify just the following single value */
#define LINKLISTEN_MILLISECONDS_2_WAIT    2000//(100) //(5000)

#define LINKLISTEN_POLL_PERIOD_MS         (10)//(10)
#define LINKLISTEN_POLL_COUNT             ( (LINKLISTEN_MILLISECONDS_2_WAIT) / (LINKLISTEN_POLL_PERIOD_MS) )

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */
static uint8_t sInit_done = 0;

/******************************************************************************
 * LOCAL FUNCTIONS
 */

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */
smplStatus_t SMPL_Reset(uint8_t (*f)(linkID_t))
{
	sInit_done = 0;
	//hwGenGpioIrqRmv(7, 1);
	return SMPL_Init(f);
}
/***********************************************************************************
 * @fn          SMPL_Init
 *
 * @brief       Initialize the SimpliciTI stack.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   Status of operation:
 */
extern void BSP_InitForCC1100(void); 
smplStatus_t SMPL_Init(uint8_t (*f)(linkID_t))
{
  smplStatus_t rc;

 if (!sInit_done)
  {
    /* set up radio. */
    MRFI_Init();
    /* initialize network */
    if ((rc=nwk_nwkInit(f)) != SMPL_SUCCESS)
    {
      return rc;
    }

    MRFI_WakeUp();
#if defined( FREQUENCY_AGILITY )
    {
      freqEntry_t chan;

      chan.logicalChan = 0;
      /* ok to set default channel explicitly now that MRFI initialized. */
      nwk_setChannel(&chan);
    }
#endif
    /* don't turn Rx on if we're an end device that isn't always on. */
#if !defined( END_DEVICE )
    MRFI_RxOn();
#endif

#if 0 // orign ver
#if defined( END_DEVICE )
    /* All except End Devices are in promiscuous mode */
    MRFI_SetRxAddrFilter((uint8_t *)nwk_getMyAddress());
    MRFI_EnableRxAddrFilter();
#endif
#else
    /* All except End Devices are in promiscuous mode */
    MRFI_SetRxAddrFilter((uint8_t *)nwk_getMyAddress());
    MRFI_EnableRxAddrFilter();
#endif

	BSP_InitForCC1100();

  }
  sInit_done = 1;

  /* Join. if no AP or Join fails that status is returned. */
  rc = nwk_join();

  return rc;
}

/******************************************************************************
 * @fn          SMPL_LinkListen
 *
 * @brief       Listen for a link frame from a 'client' device.
 *
 * input parameters
 *
 * output parameters
 * @param   linkID     - pointer to Link ID to be used by application to
 *                       read and write to the linked peer.
 *
 * @return   status of operation.
 */

smplStatus_t SMPL_LinkListen(linkID_t *linkID)
{
#if 0
  uint8_t  radioState = MRFI_GetRadioState();
  //uint16_t i;
  uint32_t i;
  linkID_t locLinkID;

  /* Set the context. We want to reject any link frames received if
   * we're not listening. For example if we're an AP we are in
   * promiscuous mode and we'll see any broadcast link frames.
   */
  nwk_setListenContext(LINK_LISTEN_ON);

  NWK_CHECK_FOR_SETRX();
//sio_printf("\n..........SMPL_LinkListen111111111\n");
  for (i=0; i<LINKLISTEN_POLL_COUNT; ++i)
  {
    /* check the semaphore. local port is assigned when the reply is sent. */
    if ((locLinkID=nwk_getLocalLinkID()))
    {
      break;
    }
    NWK_DELAY(LINKLISTEN_POLL_PERIOD_MS);
  }
//sio_printf("\n..........SMPL_LinkListen22222222\n");

  NWK_CHECK_FOR_RESTORE_IDLE();

  /* If the listen is terminated without hearing a message and setting a
   * link ID the listen context must be explicitly turned off.
   */
  if (!(locLinkID))
  {
    nwk_setListenContext(LINK_LISTEN_OFF);
    return SMPL_TIMEOUT;
  }

  *linkID = locLinkID;

  return SMPL_SUCCESS;
#else
	linkID_t locLinkID;

	locLinkID = nwk_getLocalLinkID();
	if (!(locLinkID))
	{
		return SMPL_TIMEOUT;
	}

	*linkID = locLinkID;

	return SMPL_SUCCESS;
#endif
}

/******************************************************************************
 * @fn          SMPL_Send
 *
 * @brief       Send a message to a peer application.
 *
 * input parameters
 * @param   lid     - Link ID (port) from application
 * @param   msg     - pointer to message from app to be sent
 * @param   len     - length of enclosed message
 *
 * output parameters
 *
 * @return   Status of operation. Either succeeds or it fails because of a timeout.
 *           A timeout can occur if the channel cannot be obtained. The frame
 *           buffer is discarded and the Send call must be redone by the app.
 */
smplStatus_t SMPL_Send(linkID_t lid, uint8_t *msg, uint8_t len)
{
  frameInfo_t *pFrameInfo;
  connInfo_t  *pCInfo = nwk_getConnInfo(lid);
  uint8_t      port;
  smplStatus_t rc = SMPL_BAD_PARAM;

  /* we have the connection info for this Link ID. make sure it is valid. */
   if (!pCInfo || ((rc=nwk_checkConnInfo(pCInfo, CHK_TX)) != SMPL_SUCCESS))
  {
    return rc;
  }

  /* parameter sanity check... */
  if (!msg || (len > MAX_APP_PAYLOAD))
  {
    return SMPL_BAD_PARAM;
  }

  // build an outgoing message frame destined for the port from the
  // connection info using the destination address also from the
  // connection info.
  port = pCInfo->portTx;
  if (!(pFrameInfo=nwk_buildFrame(port, msg, len, pCInfo->hops2target)))
  {
    return SMPL_NOMEM;
  }
  memcpy(MRFI_P_DST_ADDR(&pFrameInfo->mrfiPkt), pCInfo->peerAddr, NET_ADDR_SIZE);

#ifdef ACCESS_POINT
  /* If we are an AP trying to send to a polling device, don't do it.
   * See if the target is a store-and-forward client.
   */
  if (nwk_isSandFClient(MRFI_P_DST_ADDR(&pFrameInfo->mrfiPkt)))
  {
     pFrameInfo->fi_usage = FI_INUSE_UNTIL_FWD;
     return SMPL_SUCCESS;
  }
  else
#endif  /* ACCESS_POINT */
  {
    return nwk_sendFrame(pFrameInfo, MRFI_TX_TYPE_CCA);
  }
}

/**************************************************************************************
 * @fn          SMPL_Receive
 *
 * @brief       Receive a message from a peer application.
 *
 * input parameters
 * @param   lid     - Link ID (port) from application
 *
 *
 * output parameters
 * @param   msg     - pointer to where received message should be copied.
 *                    buffer should be of size == MAX_APP_PAYLOAD
 * @param   len     - pointer to receive length of received message
 *
 * @return    Status of operation.
 *            Caller should not use the value returned in 'len' to decide
 *            whether there is a frame or not. It could be useful to the
 *            Caller to distinguish between no frame and a frame with no data.
 *            For example, in the polling case a frame with no application payload
 *            is the way the AP conveys that there are no frames waiting.
 */
smplStatus_t SMPL_Receive(linkID_t lid, uint8_t *msg, uint8_t *len)
{
  connInfo_t  *pCInfo = nwk_getConnInfo(lid);
  smplStatus_t rc = SMPL_BAD_PARAM;
  rcvContext_t rcv;

  if (!pCInfo || ((rc=nwk_checkConnInfo(pCInfo, CHK_RX)) != SMPL_SUCCESS))
  {
    return rc;
  }

  rcv.type  = RCV_APP_LID;
  rcv.t.lid = lid;

#ifdef RX_POLLS
  {
    uint8_t numChans  = 1;
#if defined( FREQUENCY_AGILITY )
    freqEntry_t chans[NWK_FREQ_TBL_SIZE];
    uint8_t     scannedB4 = 0;
#endif

    do
    {
      uint8_t radioState = MRFI_GetRadioState();

      /* I'm polling. Do the poll to stimulate the sending of a frame. If the
       * frame has application length of 0 it means there were no frames.  If
       * no reply is received infer that the channel is changed. We then need
       * to scan and then retry the poll on each channel returned.
       */
      if (SMPL_SUCCESS != nwk_poll(pCInfo->portRx, pCInfo->peerAddr))
      {
        /* for some reason couldn't send the poll out. */
        return SMPL_NO_FRAME;
      }

      NWK_CHECK_FOR_SETRX();

      // do this before code block below which may reset it.
      numChans--;

      /* Wait until there's a frame. if the len is 0 then return SMPL_NO_FRAME
       * to the caller. In the poll case the AP always sends something.
       */
      {
        uint8_t spin       = NWK_RX_RETRY_COUNT;

        do
        {
          // TODO: deal with pending
          if (SMPL_SUCCESS == nwk_retrieveFrame(&rcv, msg, len, 0, 0))
          {
            break;
          }
          NWK_DELAY(WAIT_SHORT_TIME);
          spin--;
        } while (spin);

        NWK_CHECK_FOR_RESTORE_IDLE();

#if defined( FREQUENCY_AGILITY )

        if (spin)
        {
          /* we received something... */
          return (*len) ? SMPL_SUCCESS : SMPL_NO_FRAME;
        }

        /* No reply. scan for other channel(s) if we haven't already. Then set
       * one and try again.
       */
        if (!scannedB4)
        {
          numChans  = nwk_scanForChannels(chans);
          scannedB4 = 1;
        }
        if (numChans)
        {
          nwk_setChannel(&chans[numChans-1]);
        }
#else /*  FREQUENCY_AGILITY */
        return (*len) ? SMPL_SUCCESS : (spin ? SMPL_NO_FRAME : SMPL_TIMEOUT);
#endif
      }
    } while (numChans);
  }

#if defined( FREQUENCY_AGILITY )
  return SMPL_TIMEOUT;
#endif

#else  /* RX_POLLS */
  return nwk_retrieveFrame(&rcv, msg, len, 0, 0);
#endif  /* RX_POLLS */
}


/**************************************************************************************
 * @fn          SMPL_PackeCheck
 *
 * @brief       Receive a message from a peer application.
 *
 * input parameters
 * @param   lid     - Link ID (port) from application
 *
 *
 * output parameters
 * @param   msg     - pointer to where received message should be copied.
 *                    buffer should be of size == MAX_APP_PAYLOAD
 * @param   len     - pointer to receive length of received message
 *
 * @return    Status of operation.
 *            Caller should not use the value returned in 'len' to decide
 *            whether there is a frame or not. It could be useful to the
 *            Caller to distinguish between no frame and a frame with no data.
 *            For example, in the polling case a frame with no application payload
 *            is the way the AP conveys that there are no frames waiting.
 */
smplStatus_t SMPL_ReceiveNoFree(linkID_t lid, uint8_t *msg, uint8_t *len)
{
  connInfo_t  *pCInfo = nwk_getConnInfo(lid);
  smplStatus_t rc = SMPL_BAD_PARAM;
  rcvContext_t rcv;

  if (!pCInfo || ((rc=nwk_checkConnInfo(pCInfo, CHK_RX)) != SMPL_SUCCESS))
  {
    return rc;
  }

  rcv.type  = RCV_APP_LID;
  rcv.t.lid = lid;

#ifdef RX_POLLS
  {
    uint8_t numChans  = 1;
#if defined( FREQUENCY_AGILITY )
    freqEntry_t chans[NWK_FREQ_TBL_SIZE];
    uint8_t     scannedB4 = 0;
#endif

    do
    {
      uint8_t radioState = MRFI_GetRadioState();

      /* I'm polling. Do the poll to stimulate the sending of a frame. If the
       * frame has application length of 0 it means there were no frames.  If
       * no reply is received infer that the channel is changed. We then need
       * to scan and then retry the poll on each channel returned.
       */
      if (SMPL_SUCCESS != nwk_poll(pCInfo->portRx, pCInfo->peerAddr))
      {
        /* for some reason couldn't send the poll out. */
        return SMPL_NO_FRAME;
      }

      NWK_CHECK_FOR_SETRX();

      // do this before code block below which may reset it.
      numChans--;

      /* Wait until there's a frame. if the len is 0 then return SMPL_NO_FRAME
       * to the caller. In the poll case the AP always sends something.
       */
      {
        uint8_t spin       = NWK_RX_RETRY_COUNT;

        do
        {
          // TODO: deal with pending
          if (SMPL_SUCCESS == nwk_retrieveFrame(&rcv, msg, len, 0, 0))
          {
            break;
          }
          NWK_DELAY(WAIT_SHORT_TIME);
          spin--;
        } while (spin);

        NWK_CHECK_FOR_RESTORE_IDLE();

#if defined( FREQUENCY_AGILITY )

        if (spin)
        {
          /* we received something... */
          return (*len) ? SMPL_SUCCESS : SMPL_NO_FRAME;
        }

        /* No reply. scan for other channel(s) if we haven't already. Then set
       * one and try again.
       */
        if (!scannedB4)
        {
          numChans  = nwk_scanForChannels(chans);
          scannedB4 = 1;
        }
        if (numChans)
        {
          nwk_setChannel(&chans[numChans-1]);
        }
#else /*  FREQUENCY_AGILITY */
        return (*len) ? SMPL_SUCCESS : (spin ? SMPL_NO_FRAME : SMPL_TIMEOUT);
#endif
      }
    } while (numChans);
  }

#if defined( FREQUENCY_AGILITY )
  return SMPL_TIMEOUT;
#endif

#else  /* RX_POLLS */
  return nwk_retrieveFrameNoFree(&rcv, msg, len);
#endif  /* RX_POLLS */
}

/******************************************************************************
 * @fn          SMPL_Link
 *
 * @brief       Link to a peer.
 *
 * input parameters
 *
 * output parameters
 * @param   lid     - pointer to where we should write the link ID to which the
 *                    application will read and write.
 *
 * @return   Status of operation.
 */
smplStatus_t SMPL_Link(linkID_t *lid)
{
  return nwk_link(lid);
}

/*added by cbwang */
/******************************************************************************
 * @fn          SMPL_LinkClose
 *
 * @brief       close link
 *
 * input parameters
 * @param   lid     - pointer to where we should write the link ID to which the
 *                    application will read and write.
 *
 * output parameters
 *
 * @return   Status of operation.
 */
smplStatus_t SMPL_LinkClose(linkID_t lid)
{
	nwk_linkClose(lid);
	return SMPL_SUCCESS;
}
/*end*/
/******************************************************************************
 * @fn          SMPL_Ping
 *
 * @brief       Ping a peer. Synchronous call.
 *
 * input parameters
 *
 * output parameters
 * @param   lid     - The link ID to which the ping is directed
 *
 * @return   Status of operation.
 */
smplStatus_t SMPL_Ping(linkID_t lid)
{
  return nwk_ping(lid);
}

/******************************************************************************
 * @fn          SMPL_Ioctl
 *
 * @brief       This routine supplies the SimpliciTI IOCTL support.
 *
 * input parameters
 * @param   object   - The IOCTL target object
 * @param   action   - The IOCTL target action on the object
 * @param   val      - pointer to value. exact forn depends on object type.
 *
 * output parameters
 *
 * @return   Status of action. Value depends on object, action, and result.
 */
smplStatus_t SMPL_Ioctl(ioctlObject_t object, ioctlAction_t action, void *val)
{
  smplStatus_t rc = SMPL_SUCCESS;

  switch (object)
  {
    case IOCTL_OBJ_CONNOBJ:
      rc = nwk_connectionControl(action, val);
      break;

    case IOCTL_OBJ_ADDR:
      if ((IOCTL_ACT_GET == action) || (IOCTL_ACT_SET == action))
      {
        rc = nwk_deviceAddress(action, (addr_t *)val);
      }
      else
      {
        rc = SMPL_BAD_PARAM;
      }
      break;

    case IOCTL_OBJ_RAW_IO:
      if (IOCTL_ACT_WRITE == action)
      {
        rc = nwk_rawSend((ioctlRawSend_t *)val);
      }
      else if (IOCTL_ACT_READ == action)
      {
        rc = nwk_rawReceive((ioctlRawReceive_t *)val);
      }
      else
      {
        rc = SMPL_BAD_PARAM;
      }
      break;

    case IOCTL_OBJ_RADIO:
      rc = nwk_radioControl(action, val);
      break;

#ifdef ACCESS_POINT
    case IOCTL_OBJ_AP_JOIN:
      rc = nwk_joinContext(action);
      break;
#endif
#if defined( FREQUENCY_AGILITY )
    case IOCTL_OBJ_FREQ:
      rc = nwk_freqControl(action, val);
      break;
#endif
//added by cbwang 
    case IOCTL_OBJ_MANAGE:
		 if (IOCTL_ACT_CMD_RESET == action)
		 {
		 	rc = broadcast_nwk_reset();
		 }
		 else if (IOCTL_ACT_CMD_SEARCH_SERVER == action)		 	
		 {
		 	rc = broadcast_nwk_search_server();		  
		 }
		 	
		break;
//end
    case IOCTL_OBJ_CRYPTKEY:
    default:
      rc = SMPL_BAD_PARAM;
      break;
  }

  return rc;
}

