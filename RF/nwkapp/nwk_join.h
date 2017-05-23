/**************************************************************************************************
  Filename:       nwk_join.h
  Revised:        $Date: 2008-03-18 17:12:36 -0700 (Tue, 18 Mar 2008) $
  Revision:       $Revision: 16605 $
  Author:         $Author: lfriedman $

  Description:    This header file supports the SimpliciTI Join network application.

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


#ifndef NWK_JOIN_H
#define NWK_JOIN_H

#define JOIN_CONTEXT_ON  (0x01)
#define JOIN_CONTEXT_OFF (0x02)

/* place holder... */
#define SEC_CRYPT_KEY_SIZE  0

/* application payload offsets */
/*    both */
#define JB_APP_INFO_OS           0
#define JB_TID_OS                1
/*    join frame */
#define J_JOIN_TOKEN_OS          2
#define J_NUMCONN_OS             6
//added by cbwang
#define J_NODETYPE_OS             7
//end

/*    join reply frame */
#define JR_LINK_TOKEN_OS         2
#define JR_CRYPTKEY_SIZE_OS      6
#define JR_CRYPTKEY_OS           7

/* change the following as protocol developed */
#define MAX_JOIN_APP_FRAME    (JR_CRYPTKEY_OS + SEC_CRYPT_KEY_SIZE)

/* set out frame size */
#define JOIN_FRAME_SIZE         7+1 //7
#define JOIN_REPLY_FRAME_SIZE   MAX_JOIN_APP_FRAME

/* prototypes */
void         nwk_joinInit(uint8_t (*)(linkID_t));
smplStatus_t nwk_join(void);
fhStatus_t   nwk_processJoin(mrfiPacket_t *);
void         nwk_getJoinToken(uint32_t *);
uint8_t      nwk_isSandFClient(uint8_t *);
void         nwk_setJoinContext(uint8_t);

#endif
