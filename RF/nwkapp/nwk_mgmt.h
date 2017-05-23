/**************************************************************************************************
  Filename:       nwk_mgmt.h
  Revised:        $Date: 2008-03-18 17:12:36 -0700 (Tue, 18 Mar 2008) $
  Revision:       $Revision: 16605 $
  Author:         $Author: lfriedman $

  Description:    This header file supports the SimpliciTI Mgmt network application.

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

#ifndef NWK_MGMT_H
#define NWK_MGMT_H

/* MGMT frame application requests */
#define  MGMT_CMD_POLL        0x01
//added by cbwang
#define  MGMT_CMD_RESET      0x02
#define  MGMT_CMD_SEARCH_SERVER      0x03
#define  MGMT_CMD_SEARCH_SERVER_REPLY 0x04
#define  MGMT_CMD_RESEND_REQUEST 0x05
#define  MGMT_CMD_RESEND_REQUEST_REPLY	0x06
#define  MGMT_CMD_RTS	0x07
#define  MGMT_CMD_CTS	0x08
#define  MGMT_CMD_OTHER_NODE_RTS	0x09
#define  MGMT_CMD_OTHER_NODE_CTS	0x0A
#define  MGMT_CMD_OTHER_ED_NODE_RTS	0x0B
#define  MGMT_CMD_DATA_ACK	0x0C
#define  MGMT_CMD_DS	0x0D
#define  MGMT_CMD_OTHER_DATA_ACK	0x0E
#define  MGMT_CMD_OTHER_DS	0x0F

#define  M_RESEND_REQUEST_OPERATION_CODE_OS 2
#define  M_RESEND_REQUEST_TRANSCATION_ID_OS 3

#define  M_DS_CMD_OS 0
#define  M_DS_SENDER_ADDRESS_OS 1
#define  M_DS_RECEIVER_ADDRESS_OS 5
#define  M_DS_DATA_SIZE_OS	9

#define  M_RTS_CMD_OS 0
#define  M_RTS_NO_USE_OS 1
#define  M_RTS_LINK_ID_OS 2
#define  M_RTS_OPERATON_CODE_OS 3
#define  M_RTS_GUID_OS 4
#define  M_RTS_DATA_LEN_OS 8
#define  M_RTS_SRC_ADDRESS_OS 12
#define  M_RTS_DST_ADDRESS_OS 16

#define  M_CTS_CMD_OS 0
#define  M_CTS_DST_ADDRESS_OS 1
#define  M_CTS_DATA_LEN_OS	 5

#define  M_DATA_ACK_CMD_OS 0
#define  M_DATA_ACK_DST_ADDRESS_OS 1
#define  M_DATA_ACK_GUID_OS	 5


#define  M_RTS_REPLY_DATA_LEN_OS 1


#define  M_SEARCH_SERVER_NODE_TYPE_OS 2
#define  M_SEARCH_SERVER_REPLY_RSSI_OS 3
#define  M_SEARCH_SERVER_REPLY_LQI_OS 4

#define  M_RESET_NODE_TYPE_OS 2

#define  MGMT_RESET_FRAME_SIZE  3
#define  MGMT_SEARCH_SERVER_FRAME_SIZE  3
#define  MGMT_SEARCH_SERVER_REPLY_FRAME_SIZE  5
#define  MGMT_RTS_FRAME_SIZE  (20)
#define  MGMT_CTS_FRAME_SIZE  9
#define  MGMT_DATA_ACK_FRAME_SIZE  9
#define  MGMT_DS_FRAME_SIZE  (13)

//end

/* change the following as protocol developed */
#define MAX_MGMT_APP_FRAME    7

/* application payload offsets */
/*    both */
#define MB_APP_INFO_OS           0
#define MB_TID_OS                1

/*    Poll frame */
#define M_POLL_PORT_OS          2
#define M_POLL_ADDR_OS          3

/* change the following as protocol developed */
#define MAX_MGMT_APP_FRAME    7

/* frame sizes */
#define MGMT_POLL_FRAME_SIZE  7

/* prototypes */
void         nwk_mgmtInit(void);
fhStatus_t   nwk_processMgmt(mrfiPacket_t *);
smplStatus_t nwk_poll(uint8_t, uint8_t *);
//added by cbwang
smplStatus_t broadcast_nwk_reset(void);
smplStatus_t broadcast_nwk_search_server(void);
smplStatus_t search_server_reply_process(mrfiPacket_t *frame);
smplStatus_t resend_request_process(mrfiPacket_t *frame);
smplStatus_t nwk_Data_Ack_process(mrfiPacket_t *frame);
smplStatus_t nwk_DS_process(mrfiPacket_t *frame);
smplStatus_t nwk_CTS_process(mrfiPacket_t *frame);
smplStatus_t nwk_RTS_process(mrfiPacket_t *frame);
//end
#endif

