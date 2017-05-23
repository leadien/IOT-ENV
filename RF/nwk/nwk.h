/**************************************************************************************************
Filename:       nwk.h
Revised:        $Date: 2008-04-17 11:26:53 -0700 (Thu, 17 Apr 2008) $
Revision:       $Revision: 16868 $
Author:         $Author: lfriedman $

Description:    This header file supports the SimpliciTI network layer.

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

#ifndef NWK_H
#define NWK_H


/* well known ports*/
#define SMPL_PORT_PING          0x01
#define SMPL_PORT_LINK          0x02
#define SMPL_PORT_JOIN          0x03
#define SMPL_PORT_SECURITY      0x04
#define SMPL_PORT_FREQ          0x05
#define SMPL_PORT_MGMT          0x06
#define SMPL_PORT_CALL		   0x07

#define SMPL_PORT_NWK_BCAST     0x1F

#if 0
#define SMPL_PORT_USER_MAX      0x3D
#define SMPL_PORT_TX_ONLY       0x3E
#define SMPL_PORT_USER_BCAST    0x3F
#else
#define SMPL_PORT_USER_MAX      0xA0
#define SMPL_PORT_TX_ONLY       0xA1
#define SMPL_PORT_USER_BCAST    0xA2
#endif

/* Unconnected User Datagram Link ID */
#define SMPL_LINKID_USER_UUD    ((linkID_t) ~0)

#define PORT_BASE_NUMBER       0x20

/* to check connection info sanity */
#define CHK_RX   0
#define CHK_TX   1

/* return types for validating a reply frame */
#define SMPL_MY_REPLY    0
#define SMPL_A_REPLY     1
#define SMPL_NOT_REPLY   2

/* when allocating local Rx port it depends on whether the allocation
* is being done as a result of a link or a link reply
*/
#define LINK_SEND   1
#define LINK_REPLY  2

#define  CONNSTATE_FREE       (0x00)
#define  CONNSTATE_JOINED     (0x01)
#define  CONNSTATE_CONNECTED  (0x02)

#define CONN_IDEL_MAX	(10)
//added by cbwang
//#define NWK_RSSI_THRESHOLD_VALUE	(-78)
#define NWK_RSSI_THRESHOLD_VALUE	(200)//(-66)
#define NWK_RSSI_STRONG_VALUE		(208)//(-60)
//end

typedef struct
{
	volatile uint8_t     connState;
	uint8_t     hops2target;
	uint8_t     peerAddr[NET_ADDR_SIZE];
	rxMetrics_t sigInfo;
	uint8_t     portRx;
	uint8_t     portTx;
	linkID_t    thisLinkID;
	uint8_t idle;
	uint8_t peerNodeType;	
} connInfo_t;


/* prototypes */
void nwk_nwkReset(void);
smplStatus_t  nwk_nwkInit(uint8_t (*)(linkID_t));
connInfo_t   *nwk_getNextConnection(void);
void nwk_freeConnectionEx(uint8_t linkID);
void          nwk_freeConnection(connInfo_t *);
uint8_t       nwk_getNextClientPort(void);
connInfo_t   *nwk_getConnInfo(linkID_t port);
connInfo_t   *nwk_isLinkDuplicate(uint8_t *, uint8_t);
connInfo_t   *nwk_findAddressMatch(mrfiPacket_t *);
smplStatus_t  nwk_checkConnInfo(connInfo_t *, uint8_t);
uint8_t       nwk_isConnectionValid(mrfiPacket_t *, linkID_t *);
uint8_t       nwk_allocateLocalRxPort(uint8_t, connInfo_t *);
uint8_t       nwk_isValidReply(mrfiPacket_t *, uint8_t, uint8_t, uint8_t);
uint8_t 	   nwk_isJoined(mrfiPacket_t *);
uint8_t nwk_isLinked(mrfiPacket_t * frame);

#ifdef AP_IS_DATA_HUB
uint8_t       nwk_saveJoinedDevice(mrfiPacket_t *);
connInfo_t   *nwk_findAlreadyJoined(mrfiPacket_t *);
#endif

#endif

