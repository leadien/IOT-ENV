/**************************************************************************************************
  Filename:       nwk_mgmt.c
  Revised:        $Date: 2008-03-18 17:12:36 -0700 (Tue, 18 Mar 2008) $
  Revision:       $Revision: 16605 $
  Author:         $Author: lfriedman $

  Description:    This file supports the SimpliciTI Mgmt network application.

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

//#define DEBUG_LEVEL 4
/******************************************************************************
 * INCLUDES
 */
#include <string.h>
#include "def.h"
#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_mgmt.h"
#include "nwk_join.h"
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
#ifndef ACCESS_POINT
static addr_t const *sAPAddr;
#endif

static volatile uint8_t sTid;

/******************************************************************************
 * LOCAL FUNCTIONS
 */
static void  smpl_send_mgmt_reply(mrfiPacket_t *);
#ifdef ACCESS_POINT
static void  send_poll_reply(mrfiPacket_t *);
static void send_search_server_reply(mrfiPacket_t *frame);
#endif
typedef  struct DEV_ADDR{
	uint8_t hotel;
	uint8_t host;
	uint16_t dev;
}DEV_ADDR_t;

/******************************************************************************
 * GLOBAL VARIABLES
 */
extern void (*gs_pReset)(void);
extern  uint8_t g_MyNodeType;
#ifdef END_DEVICE
extern volatile uint8_t g_hasServer;
extern void (*g_pfSwitchServerCallBack)(addr_t *);
extern uint8_t gs_linkID;
#endif
extern volatile uint8_t g_RTS_process_flag ;
extern UINT32 (* const callBackfunc[])(UINT8 code,void *pVal,UINT32 Valsize);
/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          nwk_mgmtInit
 *
 * @brief       Initialize Management functions.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   void
 */

void nwk_mgmtInit(void)
{
  sTid = MRFI_RandomByte();

  return;
}

/******************************************************************************
 * @fn          nwk_processMgmt
 *
 * @brief       Process Management frame. Just save the frame for the Management
 *              app it it is a reply. If it isn't a reply, send the reply in this
 *              thread.
 *
 * input parameters
 * @param   frame   - pointer to frame to be processed
 *
 * output parameters
 *
 * @return   Non-zero if frame should be kept otherwise 0 to delete frame.
 */
fhStatus_t nwk_processMgmt(mrfiPacket_t *frame)
{
  fhStatus_t   rc;
  uint8_t      replyType;

  /* If we sent this then this is the reply. Validate the
   * packet for reception by client app. If we didn't send
   * it then we are the target. send the reply.
   */
   //printf_0("nwk_processMgmt\n");
  if (SMPL_MY_REPLY == (replyType=nwk_isValidReply(frame, sTid, MB_APP_INFO_OS, MB_TID_OS)))
  {
    /* It's a match and it's a reply. Validate the received packet by
     * returning a 1 so it can be received by the client app.
     */
     printf_0(" \n nwk SMPL_MY_REPLY!!\n ");
    rc = FHS_KEEP;
  }
#if !defined( END_DEVICE )
  else if (SMPL_A_REPLY == replyType)
  {
    /* no match. if i'm not an ED this is a reply that should be passed on. */
	//printf_0(" \n get a replay !!\n ");
    rc = FHS_REPLAY;
  }
#endif  /* !END_DEVICE */
  else
  {
    /* no, we didn't send it. send reply if it's intended for us */
    //if (!memcmp(MRFI_P_DST_ADDR(frame), nwk_getMyAddress(), NET_ADDR_SIZE))
    //   printf_0("nwk_processMgmt1111\n");
    if ((!memcmp(MRFI_P_DST_ADDR(frame), nwk_getMyAddress(), NET_ADDR_SIZE)) || (!memcmp(MRFI_P_DST_ADDR(frame), nwk_getBCastAddress(), NET_ADDR_SIZE)))
    {
   // printf_0("nwk_processMgmt222\n");
      smpl_send_mgmt_reply(frame);

      /* we're done with the frame. */
      rc = FHS_RELEASE;
    }
    else
    {
        printf_0("====nwk_processMgmt=555======\n");
      rc = FHS_REPLAY;
    }
  }

  (void) replyType;  /* keep compiler happy */

  return rc;
}

static void smpl_send_mgmt_reply(mrfiPacket_t *frame)
{
#ifdef ACCESS_POINT
	uint8_t srcNodeType;
#endif

#if 0 //orign ver
#ifdef ACCESS_POINT
  /* what kind of management frame is this? */
  switch (*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+MB_APP_INFO_OS))
  {
    case MGMT_CMD_POLL:
      send_poll_reply(frame);
      break;
//added by cbwang	  
    case MGMT_CMD_RESET:
	sio_printf1("MGMT_CMD_RESET \n");
	if(gs_pReset){
	   gs_pReset();
	}
	break;
    case MGMT_CMD_SEARCH_SERVER:
	sio_printf1("MGMT_CMD_SEARCH_SERVER \n");
	send_search_server_reply(frame);
	break;
//end		
    default:
      break;
  }
#endif  /* ACCESS_POINT */
#else
  /* what kind of management frame is this? */
  switch (*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+MB_APP_INFO_OS))
  {
#ifdef ACCESS_POINT  
    case MGMT_CMD_POLL:
      send_poll_reply(frame);
      break;
//added by cbwang	  
    case MGMT_CMD_RESET:
	{
		printf_0("MGMT_CMD_RESET \n");

		srcNodeType = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+ M_RESET_NODE_TYPE_OS);
		if ( srcNodeType > g_MyNodeType )
		{
			if(gs_pReset)
			{
				gs_pReset();
			}
		}
	}
	break;
    case MGMT_CMD_SEARCH_SERVER:
	printf_1("MGMT_CMD_SEARCH_SERVER \n");
	send_search_server_reply(frame);
	break;
//end
#endif  /* ACCESS_POINT */	
#ifdef END_DEVICE
	case MGMT_CMD_SEARCH_SERVER_REPLY:
		printf_1("MGMT_CMD_SEARCH_SERVER_REPLY!!! \n");		
		search_server_reply_process(frame);
		break;
#endif
	case MGMT_CMD_RESEND_REQUEST:
		//printf_1("MGMT_CMD_RESEND_REQUEST \n");		
		resend_request_process(frame);
		break;
	case MGMT_CMD_RTS:
		nwk_RTS_process(frame);
		break;
	case MGMT_CMD_CTS:
		nwk_CTS_process(frame);
		break;
	case MGMT_CMD_DS:		
		nwk_DS_process(frame);
		break;		
	case MGMT_CMD_DATA_ACK:
		nwk_Data_Ack_process(frame);
		break;		
    default:
      break;
  }

#endif
  return;
}


#ifdef ACCESS_POINT
static void send_poll_reply(mrfiPacket_t *frame)
{
  frameInfo_t *pOutFrame;

  /* Make sure this guy is really a client. We can tell from the source address. */
  if (!nwk_isSandFClient(MRFI_P_SRC_ADDR(frame)))
  {
    /* TODO: maybe send an error frame? */
    return;
  }

  if ( NULL != (pOutFrame = nwk_getSandFFrame(frame, M_POLL_PORT_OS)))
  {
    // reset hop count...
    PUT_INTO_FRAME(MRFI_P_PAYLOAD(&pOutFrame->mrfiPkt), F_HOP_COUNT, MAX_HOPS_FROM_AP);
    nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED);
  }
  else
  {
    nwk_SendEmptyPollRspFrame(frame);
  }

  return;
}

//added by cbwang 
extern uint16_t g_WERSSystemId;
#define S_SEARCH_SERVER_TOKEN_OS	2
static void send_search_server_reply(mrfiPacket_t *frame)
{
	frameInfo_t *pOutFrame;
	uint8_t      msg[MGMT_SEARCH_SERVER_REPLY_FRAME_SIZE];
  	uint16_t	 systemId;
	uint8_t SRCNodeType;
	int8_t rssi;
	
	memcpy(&systemId, MRFI_P_SRC_ADDR(frame), 2);
	printf_1("===send_search_server_reply  111111  srcsystemId: %d   dstsystemID:%d\n",systemId,g_WERSSystemId);

	SRCNodeType = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+ M_SEARCH_SERVER_NODE_TYPE_OS);
		
	if(systemId != g_WERSSystemId)
		return;
	printf_1("===send_search_server_reply  222\n");


	rssi = frame->rxMetrics[MRFI_RX_METRICS_RSSI_OFS];
	if ( RF_ED != g_MyNodeType 
		&& RF_ED == SRCNodeType 
		)//&& rssi > NWK_RSSI_THRESHOLD_VALUE )	
	{
		msg[MB_APP_INFO_OS] = MGMT_CMD_SEARCH_SERVER_REPLY;
		msg[MB_TID_OS]      = sTid;
		msg[M_SEARCH_SERVER_NODE_TYPE_OS] = g_MyNodeType;
		msg[M_SEARCH_SERVER_REPLY_RSSI_OS] = rssi;                     // RSSI
		msg[M_SEARCH_SERVER_REPLY_LQI_OS] =  frame->rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS];;		// LQI
	
		if ( NULL != (pOutFrame = nwk_buildFrame(SMPL_PORT_MGMT, msg, sizeof(msg), MAX_HOPS_FROM_AP)))
		{
			/* destination address is the source adddress of the received frame. */
			memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);

			nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_CCA);
		}
	}
	return;
}


smplStatus_t resend_request_process(mrfiPacket_t *frame)
{
	UINT8 msg[3];
	UINT8* plinkID = &msg[0];
	UINT8* pOperationCode = &msg[1];
	UINT8* pTranscationID = &msg[2];
	
	smplStatus_t ret = SMPL_SUCCESS;
	
	if ( NULL == frame )
	{
		ret = SMPL_BAD_PARAM;
	}
	else
	{
		if ( nwk_isConnectionValid(frame,plinkID) ) 
		{
			*pOperationCode = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+ M_RESEND_REQUEST_OPERATION_CODE_OS); 
			*pTranscationID = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+ M_RESEND_REQUEST_TRANSCATION_ID_OS); 

			//printf_0("  msg:{ %d, %d, %d }\n",msg[0],msg[1],msg[2]);
			//if ( (sizeof(callBackfunc)/sizeof(callBackfunc[0])) > 0 )
			if(callBackfunc[0])
			{
				callBackfunc[0](MGMT_CMD_RESEND_REQUEST,msg,sizeof(msg));
			}
		}
		else
		{
			//printf_0("  NO LINK\n");
			ret = SMPL_NO_LINK;
		}
	}
	

	return ret;
}


//end
#else  // ACCESS_POINT

smplStatus_t nwk_poll(uint8_t port, uint8_t *addr)
{
  uint8_t        msg[MGMT_POLL_FRAME_SIZE];
  ioctlRawSend_t send;

  msg[MB_APP_INFO_OS] = MGMT_CMD_POLL;
  msg[MB_TID_OS]      = sTid;
  msg[M_POLL_PORT_OS] = port;
  memcpy(msg+M_POLL_ADDR_OS, addr, NET_ADDR_SIZE);

  if (!sAPAddr)
  {
    sAPAddr = nwk_getAPAddress();
    if (!sAPAddr)
    {
      return SMPL_NO_CHANNEL;
    }
  }
  send.addr = (addr_t *)sAPAddr;
  send.msg  = msg;
  send.len  = sizeof(msg);
  send.port = SMPL_PORT_MGMT;

  return SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
}

#endif /* ACCESS_POINT */



//added by cwbang
smplStatus_t broadcast_nwk_reset(void)
{
  uint8_t        msg[MGMT_RESET_FRAME_SIZE];
  ioctlRawSend_t send;

  msg[MB_APP_INFO_OS] = MGMT_CMD_RESET;
  msg[MB_TID_OS]      = sTid;
  msg[M_RESET_NODE_TYPE_OS] = g_MyNodeType;

  send.addr = (addr_t *)nwk_getBCastAddress();
  send.msg  = msg;
  send.len  = sizeof(msg);
  send.port = SMPL_PORT_MGMT;

  return SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
}

smplStatus_t broadcast_nwk_search_server(void)
{
  uint8_t        msg[MGMT_SEARCH_SERVER_FRAME_SIZE];
  ioctlRawSend_t send;

  msg[MB_APP_INFO_OS] = MGMT_CMD_SEARCH_SERVER;
  msg[MB_TID_OS]      = sTid;
  msg[M_SEARCH_SERVER_NODE_TYPE_OS] = g_MyNodeType;

  send.addr = (addr_t *)nwk_getBCastAddress();
  send.msg  = msg;
  send.len  = sizeof(msg);
  send.port = SMPL_PORT_MGMT;
	
  return SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
}


smplStatus_t nwk_Data_Ack_process(mrfiPacket_t *frame)
{
	uint8_t *pRTSDstAddress;
	smplStatus_t ret = SMPL_SUCCESS;
	uint8_t code;
	
	if ( NULL != frame )
	{
		pRTSDstAddress = (MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_DATA_ACK_DST_ADDRESS_OS);	
		if (!memcmp(pRTSDstAddress, nwk_getMyAddress(), NET_ADDR_SIZE)) 
		{
			code = MGMT_CMD_DATA_ACK;
		}
		else
		{
			code = MGMT_CMD_OTHER_DATA_ACK;
		}
		
		//for me		
		if (  callBackfunc[1] )
		{
			callBackfunc[1](code,(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS),MGMT_DATA_ACK_FRAME_SIZE);
		}
	}

	return ret;
}

smplStatus_t nwk_DS_process(mrfiPacket_t *frame)
{
	//uint8_t *pDSReceiverAddress;
	smplStatus_t ret = SMPL_SUCCESS;
	
	if ( NULL != frame )
	{
		//printf_0("==DS\n");
		#if 0
		pDSReceiverAddress = (MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_DS_RECEIVER_ADDRESS_OS);	
		if (!memcmp(pDSReceiverAddress, nwk_getMyAddress(), NET_ADDR_SIZE)) 
		{
			//for me	
			code = MGMT_CMD_DS;
		}
		else
		{
			code = MGMT_CMD_OTHER_DS;
			//code = MGMT_CMD_DS;
		}
		#endif
		if (  callBackfunc[1] )
		{
			UINT32 datasize;
			memcpy((UINT8*)&datasize,(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_DS_DATA_SIZE_OS),sizeof(UINT32));
			
			ret = (smplStatus_t)callBackfunc[1](MGMT_CMD_DS,&datasize,sizeof(datasize));
		}
	}
	return ret;
}


smplStatus_t nwk_CTS_process(mrfiPacket_t *frame)
{
	uint8_t *pRTSDstAddress;
	uint8_t   msg[MGMT_DS_FRAME_SIZE];
	ioctlRawSend_t send;  
	smplStatus_t ret = SMPL_SUCCESS;
	
	if ( NULL != frame )
	{
		pRTSDstAddress = (MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_CTS_DST_ADDRESS_OS);	
		if (!memcmp(pRTSDstAddress, nwk_getMyAddress(), NET_ADDR_SIZE)) 
		{
			//for me		
			if (  callBackfunc[1] )
			{
				callBackfunc[1](MGMT_CMD_CTS,NULL,0);
			}

			// boardcast ds packet						
			msg[M_DS_CMD_OS] = MGMT_CMD_DS;

			//pack dst address
			memcpy(msg+M_DS_SENDER_ADDRESS_OS,nwk_getMyAddress(),sizeof(addr_t));
			memcpy(msg+M_DS_RECEIVER_ADDRESS_OS,MRFI_P_SRC_ADDR(frame),sizeof(addr_t));
			memcpy(msg+M_DS_DATA_SIZE_OS,(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_CTS_DATA_LEN_OS),sizeof(UINT32));

			send.addr = (addr_t *)nwk_getBCastAddress();
			send.msg  = msg;
			send.len  = sizeof(msg);
			send.port = SMPL_PORT_MGMT;

			ret = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
		}
		else
		{
			// not for me	to notify wait ..
			if (  callBackfunc[1] )
			{
				UINT32 datalen;
				memcpy((UINT8*)&datalen,(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_CTS_DATA_LEN_OS),sizeof(UINT32));
				callBackfunc[1](MGMT_CMD_OTHER_NODE_CTS,&datalen,sizeof(datalen));
			}
		}
	}

	return ret;
}

#if 1

smplStatus_t nwk_RTS_process(mrfiPacket_t *frame)
{ 
//	ioctlRawSend_t send;  
	uint8_t *pRTSDstAddress;

	smplStatus_t ret = SMPL_SUCCESS;
	//printf_0("===nwk_RTS_process===\n");
	if ( NULL != frame )
	{
		pRTSDstAddress = (MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_RTS_DST_ADDRESS_OS);	
		if (!memcmp(pRTSDstAddress, nwk_getMyAddress(), NET_ADDR_SIZE)) 
		{	
			// for me and relpy
 			printf_1("===nwk_RTS_Reply===\n"); 
			
			if (  callBackfunc[1] )
			{
				//printf_0("===nwk_RTS_process=333==\n");
				
				UINT8 *pLinkID = (MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_RTS_LINK_ID_OS);
				
				if ( nwk_isConnectionValid(frame,pLinkID) ) 
				{				
					//printf_0("===nwk_RTS_process=444==\n");
					 callBackfunc[1](MGMT_CMD_RTS,(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS),MGMT_RTS_FRAME_SIZE);
					 
				}
			}				
		}
		else
		{
			// not for me	to notify wait ..
			if (  callBackfunc[1] )
			{
				UINT32 datalen;
				memcpy((UINT8*)&datalen,(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_RTS_DATA_LEN_OS),sizeof(UINT32));
				callBackfunc[1](MGMT_CMD_OTHER_NODE_RTS,&datalen,sizeof(datalen));
			}			
		}
	}
	
	return ret;
}


#else

smplStatus_t nwk_RTS_process(mrfiPacket_t *frame)
{
	uint8_t        msg[MGMT_CTS_FRAME_SIZE];
	ioctlRawSend_t send;  
	uint8_t *pRTSDstAddress;
	smplStatus_t ret = SMPL_SUCCESS;
	//printf_0("===nwk_RTS_process===\n");
	if ( NULL != frame )
	{
		pRTSDstAddress = (MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_RTS_CTS_DST_ADDRESS_OS);	
		if (!memcmp(pRTSDstAddress, nwk_getMyAddress(), NET_ADDR_SIZE)) 
		{	
			// for me and relpy
			//printf_0("===nwk_RTS_process=111==\n");
			printf_0("===nwk_RTS_Reply===\n");
			//if ( g_RTS_process_flag )			
			//printf_0("===nwk_RTS_process=22==\n");
			if (  callBackfunc[1] )
			{
			//printf_0("===nwk_RTS_process=333==\n");
				UINT8 *pLinkID = (MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_RTS_LINK_ID_OS);
				if ( nwk_isConnectionValid(frame,pLinkID) ) 
				{				
				//printf_0("===nwk_RTS_process=444==\n");
					if ( 1 == callBackfunc[1](MGMT_CMD_RTS,(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS),MGMT_RTS_FRAME_SIZE) )
					{	
						//if ( g_RTS_process_flag )		
						{
							// reply CTS to sender
							
							msg[MB_APP_INFO_OS] = MGMT_CMD_CTS;	

							//pack dst address
							memcpy(msg+M_RTS_CTS_DST_ADDRESS_OS,MRFI_P_SRC_ADDR(frame),sizeof(addr_t));
							
							//pack data len
							memcpy(msg + M_RTS_CTS_DATA_LEN_OS,(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_RTS_CTS_DATA_LEN_OS),sizeof(UINT32));

							send.addr = (addr_t *)nwk_getBCastAddress();
							send.msg  = msg;
							send.len  = sizeof(msg);
							send.port = SMPL_PORT_MGMT;

							ret = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
						}
					}
				}
			}	
		}
		else
		{
			// not for me	to notify wait ..
			if (  callBackfunc[1] )
			{
				UINT32 datalen;
				memcpy((UINT8*)&datalen,(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS + M_RTS_CTS_DATA_LEN_OS),sizeof(UINT32));
				callBackfunc[1](MGMT_CMD_OTHER_NODE_RTS,&datalen,sizeof(datalen));
			}			
		}
	}
	
	return ret;
}
#endif


#ifdef END_DEVICE
smplStatus_t search_server_reply_process(mrfiPacket_t *frame)
{
	//uint8_t replyType;
	uint8_t new_server_rssi;
	uint8_t new_server_lqi;
	uint8_t current_linked_server_rssi;
	uint8_t current_linked_server_lqi;
	uint8_t serverNodeType;
	uint8_t currentLinkNodeType;
	uint8_t switchFlag = 0;
	uint8_t	addr[NET_ADDR_SIZE];

	printf_0("===search_server_reply_process=======\n");

	connInfo_t  *pCInfo = nwk_getConnInfo(gs_linkID);
	if ( pCInfo )	
	{
		// other server,not current link server
		if (memcmp(MRFI_P_SRC_ADDR(frame), pCInfo->peerAddr, NET_ADDR_SIZE))
		{

			
			g_hasServer = 1;
			//current_linked_server_rssi = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+ M_SEARCH_SERVER_REPLY_RSSI_OS);
			//current_linked_server_lqi = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+ M_SEARCH_SERVER_REPLY_LQI_OS);

			current_linked_server_rssi = pCInfo->sigInfo.rssi;
			current_linked_server_lqi = pCInfo->sigInfo.lqi;	
			
			new_server_rssi = frame->rxMetrics[MRFI_RX_METRICS_RSSI_OFS]; 
			new_server_lqi = frame->rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS]; 
			serverNodeType = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+ M_SEARCH_SERVER_NODE_TYPE_OS);
			memcpy(addr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);
			//			
			currentLinkNodeType = pCInfo->peerNodeType;

			//sio_printf("==search 1111===addr:[ %02x %02x %02x %02x ],  rssi:%d, lqi:%d, serverNodeType:%d, currentLinkNodeType:%d  --  (rssi: %d lqi:%d)\n",addr[0],addr[1],addr[2],addr[3],new_server_rssi,new_server_lqi,serverNodeType,currentLinkNodeType, pCInfo->sigInfo.rssi, pCInfo->sigInfo.lqi);

			switch(currentLinkNodeType)
			{
				case RF_HOST_AP:
					if (  RF_HOST_AP == serverNodeType )
					{
						if (  new_server_rssi  >  current_linked_server_rssi  )
						{
							switchFlag = 1;
						}
					}
					else if (  RF_AP_TYPE_1 == serverNodeType )
					{
						//if (  rssi >  pCInfo->sigInfo.rssi + 6)
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{
							if ( new_server_rssi > NWK_RSSI_STRONG_VALUE )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if (  new_server_rssi  >  current_linked_server_rssi + 8 )
							{
								switchFlag = 1;
							}
						}	
					}
					else if (  RF_AP_TYPE_2 == serverNodeType )
					{
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{
							if ( new_server_rssi > NWK_RSSI_STRONG_VALUE + 3 )
							{
								switchFlag = 1;
							}
						}
						else
					{
							if (  new_server_rssi  >  current_linked_server_rssi + 15 )
						{
							switchFlag = 1;
						}
					}
					}
					else if (  RF_AP_TYPE_3 == serverNodeType )
					{
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{
							if ( new_server_rssi > NWK_RSSI_STRONG_VALUE + 6 )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if (  new_server_rssi  >  current_linked_server_rssi + 25 )
							{
								switchFlag = 1;
							}
						}	
					}
					else if (  RF_AP_TYPE_4 == serverNodeType )
					{
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{
							if ( new_server_rssi > NWK_RSSI_STRONG_VALUE + 9 )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if (  new_server_rssi  >  current_linked_server_rssi + 35 )
							{
								switchFlag = 1;
							}
						}	
					}
					break;
				case RF_AP_TYPE_1:
					if (  RF_HOST_AP == serverNodeType )
					{
						//if ( pCInfo->sigInfo.rssi < NWK_RSSI_STRONG_VALUE && (rssi > (NWK_RSSI_THRESHOLD_VALUE + 5) || rssi > pCInfo->sigInfo.rssi) )
						if ( new_server_rssi >=  current_linked_server_rssi )
						{
							switchFlag = 1;
						}
					}
					else if (  RF_AP_TYPE_1 == serverNodeType )
					{
						//if ( rssi > pCInfo->sigInfo.rssi && rssi > NWK_RSSI_THRESHOLD_VALUE )
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{				
							if ( new_server_rssi >  current_linked_server_rssi )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi + 5 )
						{
							switchFlag = 1;
						}
					}
					}
					else if (  RF_AP_TYPE_2 == serverNodeType )
					{
						//if ( rssi > pCInfo->sigInfo.rssi && rssi > NWK_RSSI_THRESHOLD_VALUE + 5 )
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{	
							if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi >= 8 ) // > 8
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 2 )
								{
									switchFlag = 1;
								}
							}
							else if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi >= 3 ) // 3 < and < 8
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 4 )
								{
									switchFlag = 1;
								}
							}
							else
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 6 )
								{
									switchFlag = 1;
								}
							}
							
							
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi + 15 )
							{
								switchFlag = 1;
							}
						}
					}
					else if (  RF_AP_TYPE_3 == serverNodeType )
					{
						//if ( rssi > pCInfo->sigInfo.rssi && rssi > NWK_RSSI_THRESHOLD_VALUE + 5 )
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{	
							if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi > 10 ) // > 10
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 2 )
								{
									switchFlag = 1;
								}
							}
							if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi > 6 ) //   6 < and < 10
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 3 )
								{
									switchFlag = 1;
								}
							}
							else if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi >= 3 ) // 3 < and < 6
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 6 )
								{
									switchFlag = 1;
								}
							}
							else
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 8 )
								{
									switchFlag = 1;
								}
							}
							
							
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi + 25 )
							{
								switchFlag = 1;
							}
						}
					}
					else if (  RF_AP_TYPE_4 == serverNodeType )
					{
						//if ( rssi > pCInfo->sigInfo.rssi && rssi > NWK_RSSI_THRESHOLD_VALUE + 5 )
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{	
							if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi > 10 ) // > 10
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 3 )
						{
							switchFlag = 1;
						}
					}
							if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi > 6 ) //   6 < and < 10
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 4 )
								{
									switchFlag = 1;
								}
							}
							else if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi >= 3 ) // 3 < and < 6
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 8 )
								{
									switchFlag = 1;
								}
							}
							else
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 10 )
								{
									switchFlag = 1;
								}
							}
							
							
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi + 35 )
							{
								switchFlag = 1;
							}
						}
					}
					break;
				case RF_AP_TYPE_2:
					if (  RF_HOST_AP == serverNodeType )
					{
						if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE + 6)
						{						
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 10 )
							     )
							{
								switchFlag = 1;
							}
						}
						else if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE )
						{
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 6 )
							     )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi )
							{
								switchFlag = 1;
							}
						}
					}
					else if (  RF_AP_TYPE_1 == serverNodeType )
					{
						if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE + 6)
						{						
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 10 )
							     )
							{
								switchFlag = 1;
							}
						}
						else if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE )
						{
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 6 )
							     )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi )
							{
								switchFlag = 1;
							}
						}
					}
					else if (  RF_AP_TYPE_2 == serverNodeType )
					{
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{				
							if ( new_server_rssi >  current_linked_server_rssi )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi + 5 )
							{
								switchFlag = 1;
							}
						}
					}
					else if (  RF_AP_TYPE_3 == serverNodeType )
					{
						//if ( rssi > pCInfo->sigInfo.rssi && rssi > NWK_RSSI_THRESHOLD_VALUE + 5 )
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{	
							if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi >= 8 ) // > 8
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 2 )
								{
									switchFlag = 1;
								}
							}
							else if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi >= 3 ) // 3 < and < 8
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 4 )
								{
									switchFlag = 1;
								}
							}
							else
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 6 )
								{
									switchFlag = 1;
								}
							}
							
							
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi + 15 )
							{
								switchFlag = 1;
							}
						}
					}
					else if (  RF_AP_TYPE_4 == serverNodeType )
					{
						//if ( rssi > pCInfo->sigInfo.rssi && rssi > NWK_RSSI_THRESHOLD_VALUE + 5 )
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{	
							if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi > 10 ) // > 10
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 2 )
								{
									switchFlag = 1;
								}
							}
							if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi > 6 ) //   6 < and < 10
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 3 )
								{
									switchFlag = 1;
								}
							}
							else if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi >= 3 ) // 3 < and < 6
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 6 )
								{
									switchFlag = 1;
								}
							}
							else
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 8 )
								{
									switchFlag = 1;
								}
							}
							
							
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi + 25 )
							{
								switchFlag = 1;
							}
						}
					}
					break;
				case RF_AP_TYPE_3:
					if (  RF_HOST_AP == serverNodeType )
					{
						if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE + 6)
						{						
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 10 )
							     )
							{
								switchFlag = 1;
							}
						}
						else if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE )
						{
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 6 )
							     )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi )
							{
								switchFlag = 1;
							}
						}
					}
					else if (  RF_AP_TYPE_1 == serverNodeType || RF_AP_TYPE_2 == serverNodeType )
					{
						if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE + 6)
						{						
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 10 )
							     )
							{
								switchFlag = 1;
							}
						}
						else if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE )
						{
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 6 )
							     )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi )
							{
								switchFlag = 1;
							}
						}
					}
					else if (  RF_AP_TYPE_3 == serverNodeType )
					{
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{				
							if ( new_server_rssi >  current_linked_server_rssi )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi + 5 )
							{
								switchFlag = 1;
							}
						}
					}
					else if ( RF_AP_TYPE_4 == serverNodeType )
					{
						//if ( rssi > pCInfo->sigInfo.rssi && rssi > NWK_RSSI_THRESHOLD_VALUE + 5 )
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{	
							if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi >= 8 ) // > 8
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 2 )
								{
									switchFlag = 1;
								}
							}
							else if ( NWK_RSSI_STRONG_VALUE - current_linked_server_rssi >= 3 ) // 3 < and < 8
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 4 )
								{
									switchFlag = 1;
								}
							}
							else
							{
								if ( (new_server_rssi - current_linked_server_rssi ) > 6 )
								{
									switchFlag = 1;
								}
							}
							
							
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi + 15 )
							{
								switchFlag = 1;
							}
						}
					}	
					break;
				case RF_AP_TYPE_4:
					if (  RF_HOST_AP == serverNodeType )
					{
						if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE + 6)
						{						
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 10 )
							     )
							{
								switchFlag = 1;
							}
						}
						else if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE )
						{
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 6 )
							     )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi )
							{
								switchFlag = 1;
							}
						}
					}
					else if (  RF_AP_TYPE_1 == serverNodeType || RF_AP_TYPE_2 == serverNodeType || RF_AP_TYPE_3 == serverNodeType )
					{
						if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE + 6)
						{						
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 10 )
							     )
							{
								switchFlag = 1;
							}
						}
						else if ( current_linked_server_rssi > NWK_RSSI_STRONG_VALUE )
						{
							if ( new_server_rssi >=  current_linked_server_rssi 
							     || new_server_rssi < (current_linked_server_rssi - 6 )
							     )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi )
							{
								switchFlag = 1;
							}
						}
					}
					else if (  RF_AP_TYPE_4 == serverNodeType )
					{
						if ( current_linked_server_rssi < NWK_RSSI_STRONG_VALUE )
						{				
							if ( new_server_rssi >  current_linked_server_rssi )
							{
								switchFlag = 1;
							}
						}
						else
						{
							if ( new_server_rssi >=  current_linked_server_rssi + 5 )
							{
								switchFlag = 1;
							}
						}
					}
					break;
				default:
					break;
			}

			if ( switchFlag )
			{
				sio_printf("===switchFlag====\n");
				if ( g_pfSwitchServerCallBack )
				{
					//sio_printf("===g_pfSwitchServerCallBack====\n");
					g_pfSwitchServerCallBack((addr_t*)addr);
				}
			}
		
		}

	}
	return SMPL_SUCCESS;
}
#endif
//end
