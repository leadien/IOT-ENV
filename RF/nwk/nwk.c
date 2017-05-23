
/**************************************************************************************************
  Filename:       nwk.c
  Revised:        $Date: 2008-04-17 11:26:53 -0700 (Thu, 17 Apr 2008) $
  Revision:       $Revision: 16868 $
  Author          $Author: lfriedman $

  Description:    This file supports the SimpliciTI network layer.

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
#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_app.h"
#include "nwk_globals.h"
#include "nwk_QMgmt.h"

/******************************************************************************
 * MACROS
 */
/************************* NETWORK MANIFEST CONSTANT SANITY CHECKS ****************************/
#if !defined(ACCESS_POINT) && !defined(RANGE_EXTENDER) && !defined(END_DEVICE)
#error ERROR: No SimpliciTI device type defined
#endif

#if defined( RX_LISTENS )
#error ERROR: End Device Rx-type RX_LISTENS not yet supported
#endif

#if defined( RX_NEVER )
#error ERROR: End Device Rx-type RX_NEVER not yet supported
#endif

#if defined(END_DEVICE) && !defined(RX_LISTENS) && !defined(RX_POLLS) && !defined(RX_NEVER) && !defined(RX_ALWAYS)
#error ERROR: End Device requires an Rx-type definition
#endif

#ifndef MAX_HOPS
#define MAX_HOPS  3
#elif MAX_HOPS > 4
#error ERROR: MAX_HOPS must be 4 or fewer
#endif

#ifndef MAX_APP_PAYLOAD
#error ERROR: MAX_APP_PAYLOAD must be defined
#endif

#if ( MAX_APP_PAYLOAD < MAX_FREQ_APP_FRAME )
#error ERROR: Application payload size too small for Frequency frame
#endif

#if ( MAX_APP_PAYLOAD < MAX_JOIN_APP_FRAME )
#error ERROR: Application payload size too small for Join frame
#endif

#if ( MAX_APP_PAYLOAD < MAX_LINK_APP_FRAME )
#error ERROR: Application payload size too small for Link frame
#endif

#if ( MAX_APP_PAYLOAD < MAX_MGMT_APP_FRAME )
#error ERROR: Application payload size too small for Management frame
#endif

#if ( MAX_APP_PAYLOAD < MAX_SEC_APP_FRAME )
#error ERROR: Application payload size too small for Security frame
#endif

#if ( MAX_APP_PAYLOAD < MAX_PING_APP_FRAME )
#error ERROR: Application payload size too small for Ping frame
#endif

#if NWK_FREQ_TBL_SIZE < 1
#error ERROR: NWK_FREQ_TBL_SIZE must be > 0
#endif

/************************* END NETWORK MANIFEST CONSTANT SANITY CHECKS ************************/

/******************************************************************************
 * CONSTANTS AND DEFINES
 */
#define SYS_NUM_CONNECTIONS   (NUM_CONNECTIONS+1)
#if 0
#define  LINK_BASE_LINKID  1
#define  LINK_MAX_LINKID   (NUM_CONNECTIONS+LINK_BASE_LINKID-1)

#if NUM_CONNECTIONS > 0
#define  LINKID_IS_VALID(lid)   ((((lid) >= LINK_BASE_LINKID) && ((lid) <= LINK_MAX_LINKID)) || (lid == SMPL_LINKID_USER_UUD))
#else
#define  LINKID_IS_VALID(lid)   (lid == SMPL_LINKID_USER_UUD)
#endif

#if ( LINK_BASE_LINKID < 1 )
#error ERROR: Minimum Link ID value must be > 0
#endif
#endif
// increment this if the persistentContext_t structure is changed. it will help
// detect the upgrade context: any saved values will have a version with a
// lower number.
#define  CONNTABLEINFO_STRUCTURE_VERSION   1

/******************************************************************************
 * TYPEDEFS
 */
/* This structure aggregates eveything necessary to save if we want to restore
 * the connection information later.
 */
typedef struct
{
  const uint8_t    structureVersion; /* to dectect upgrades... */
        uint8_t    numConnections;   /* count includes the UUD port/link ID */
/* The next two are used to detect overlapping port assignments. When _sending_ a
 * link frame the local port is assigned from the top down. When sending a _reply_
 * the assignment is bottom up. Overlapping assignments are rejected. That said it
 * is extremely unlikely that this will ever happen. If it does the test implemented
 * here is overly cautious (it will reject assignments when it needn't). But we leave
 * it that way on the assumption that it will never happen anyway.
 */
        uint8_t    curNextLinkPort;
        uint8_t    curMaxReplyPort;
        linkID_t   nextLinkID;
/* Connection table entries last... */
        connInfo_t connStruct[SYS_NUM_CONNECTIONS];
} persistentContext_t;

/******************************************************************************
 * LOCAL VARIABLES
 */

/* This will be overwritten if we restore the structure from NV for example.
 * Note that restoring will not permit overwriting the version element as it
 * is declared 'const'.
 */
static persistentContext_t sPersistInfo = {CONNTABLEINFO_STRUCTURE_VERSION};
void (*gs_pReset)(void) = NULL;
/******************************************************************************
 * LOCAL FUNCTIONS
 */
static uint8_t map_lid2idx(linkID_t, uint8_t *);
static void    initializeConnection(connInfo_t *);

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */
void nwk_ResetSet( void (*pReset)(void) )
{
	gs_pReset = pReset;
}

/******************************************************************************
 * @fn          nwk_nwkInit
 *
 * @brief       Initialize NWK conext.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   Status of operation.
 */
smplStatus_t nwk_nwkInit(uint8_t (*f)(linkID_t))
{
  /* Truly ugly initialization because CCE won't initialize properly. Must
   * skip first const element. Yuk.
   */
  memset((((uint8_t *)&sPersistInfo)+1), 0x0, (sizeof(sPersistInfo)-1));
  /* OK. The zeroed elements are set. Now go back and do fixups...  */

  sPersistInfo.numConnections   = SYS_NUM_CONNECTIONS;
  sPersistInfo.curNextLinkPort  = SMPL_PORT_USER_MAX;
  sPersistInfo.curMaxReplyPort  = PORT_BASE_NUMBER;
  sPersistInfo.nextLinkID       = 1;

  /* initialize globals */
  nwk_globalsInit();

  /* initialize frame processing */
  nwk_frameInit(f);

  /* initialize queue manager */
  nwk_QInit();
	
  /* initialize each network application. */
  nwk_freqInit();
  nwk_pingInit();
  nwk_joinInit(f);
  nwk_mgmtInit();
  nwk_linkInit();
  nwk_securityInit();

  /* set up the last connection as the broadcast port mapped to the broadcast Link ID */
  if (CONNSTATE_FREE == sPersistInfo.connStruct[NUM_CONNECTIONS].connState)
  {
  sPersistInfo.connStruct[NUM_CONNECTIONS].connState   = CONNSTATE_CONNECTED;
  sPersistInfo.connStruct[NUM_CONNECTIONS].hops2target = MAX_HOPS;
  sPersistInfo.connStruct[NUM_CONNECTIONS].portRx      = SMPL_PORT_USER_BCAST;
  sPersistInfo.connStruct[NUM_CONNECTIONS].portTx      = SMPL_PORT_USER_BCAST;
  sPersistInfo.connStruct[NUM_CONNECTIONS].thisLinkID  = SMPL_LINKID_USER_UUD;
    /* set peer address to broadcast so it is used when Application sends to the broadcast Link ID */
  memcpy(sPersistInfo.connStruct[NUM_CONNECTIONS].peerAddr, nwk_getBCastAddress(), NET_ADDR_SIZE);
  }

  return SMPL_SUCCESS;
}

connInfo_t * nwk_getFreeConnection(void)
{
  uint8_t i;

  for (i=0; i<SYS_NUM_CONNECTIONS; ++i)
  {
    if ( sPersistInfo.connStruct[i].connState == CONNSTATE_CONNECTED )
    {
      continue;
    }
    break;
  }

  if (SYS_NUM_CONNECTIONS == i)
  {
    return (connInfo_t *)0;
  }

  return (connInfo_t *)&(sPersistInfo.connStruct[i]);
}

/******************************************************************************
 * @fn          nwk_getNextConnection
 *
 * @brief       Return the next free connection structure if on is available.
 *
 * input parameters
 *
 * output parameters
 *      The returned structure has the Rx port number populated based on the
 *      free strucure found. This is the port queried when the app wants to
 *      do a receive.
 *
 * @return   pointer to the new connInfo_t structure. NULL if there is
 *           no room in connection structure array.
 */
connInfo_t *nwk_getNextConnection()
{
  uint8_t i;

  for (i=0; i<SYS_NUM_CONNECTIONS; ++i)
  {
    if (sPersistInfo.connStruct[i].connState == CONNSTATE_CONNECTED)
    {
      continue;
    }
    break;
  }

  if (SYS_NUM_CONNECTIONS == i)
  {
    return (connInfo_t *)0;
  }

  initializeConnection(&sPersistInfo.connStruct[i]);

  return &(sPersistInfo.connStruct[i]);
}

/************************************************************************************
 * @fn          initializeConnection
 *
 * @brief       Initialize some elements of a Connection table entry.
 *
 * input parameters
 * @param   pCInfo  - pointer to Connection Table entry to initialize. The file
 *                    scope variable holding the next link ID value is also updated.
 *
 * output parameters
 * @param   pCInfo  - certain elements are set to specific values.
 *
 *
 * @return   void
 */
static void initializeConnection(connInfo_t *pCInfo)
{
  linkID_t *locLID = &sPersistInfo.nextLinkID;
  uint8_t   tmp;

    /* this element will be populated during the exchange with the peer. */
  pCInfo->portTx = 0;
  pCInfo->idle = 0;
  pCInfo->connState  =  CONNSTATE_CONNECTED;
  pCInfo->thisLinkID = *locLID;

  /* Generate the next Link ID. This isn't foolproof. If the count wraps
   * we can end up with confusing duplicates. We can protect aginst using
   * one that is already in use but we can't protect against a stale Link ID
   * remembered by an application that doesn't know its connection has been
   * torn down. The test for 0 will hopefully never be true (indicating a wrap).
   */
  (*locLID)++;

  while (!*locLID || (*locLID == SMPL_LINKID_USER_UUD) || map_lid2idx(*locLID, &tmp))
  {
    (*locLID)++;
  }

  return;
}


/*added by cbwang */
/******************************************************************************
 * @fn          nwk_freeConnectionEx
 *
 * @brief       Return the connection structure to the free pool. Currently
 *              this routine is only called when a link freame is sent and
 *              no reply is received so the freeing steps are pretty simple.
 *              But eventually this will be more complex so this place-holder
 *              is introduced.
 *
 * input parameters
 * @param   linkID    - pointer to entry to be freed
 *
 * output parameters
 *
 * @return   None.
 */
void nwk_freeConnectionEx(uint8_t linkID)
{
	uint8_t i;
 	connInfo_t *ptr = sPersistInfo.connStruct;

	for (i=0; i<SYS_NUM_CONNECTIONS; ++i,++ptr)
	{
		if ( ptr->thisLinkID == linkID )
		{
			printf_0("link %d had closed",linkID);
			nwk_freeConnection(ptr);
			sPersistInfo.curNextLinkPort = SMPL_PORT_USER_MAX;
			sPersistInfo.curMaxReplyPort = PORT_BASE_NUMBER;
			return;
		}
	}
}
/*end*/

/******************************************************************************
 * @fn          nwk_freeConnection
 *
 * @brief       Return the connection structure to the free pool. Currently
 *              this routine is only called when a link freame is sent and
 *              no reply is received so the freeing steps are pretty simple.
 *              But eventually this will be more complex so this place-holder
 *              is introduced.
 *
 * input parameters
 * @param   pCInfo    - pointer to entry to be freed
 *
 * output parameters
 *
 * @return   None.
 */
void nwk_freeConnection(connInfo_t *pCInfo)
{
#if NUM_CONNECTIONS > 0
  pCInfo->connState = CONNSTATE_FREE;
#endif
}

/******************************************************************************
 * @fn          nwk_getConnInfo
 *
 * @brief       Return the connection info structure to which the input Link ID maps.
 *
 * input parameters
 * @param   port    - port for which mapping desired
 *
 * output parameters
 *
 * @return   pointer to connInfo_t structure found. NULL if no mapping
 *           found or entry not valid.
 */
connInfo_t *nwk_getConnInfo(linkID_t linkID)
{
  uint8_t idx, rc;
#if 0
  if (!LINKID_IS_VALID(linkID))
  {
    return (connInfo_t *)0;
  }
#endif
  rc = map_lid2idx(linkID, &idx);

  return (rc && (CONNSTATE_CONNECTED == sPersistInfo.connStruct[idx].connState)) ? &(sPersistInfo.connStruct[idx]) : (connInfo_t *)0;
}

/******************************************************************************
 * @fn          nwk_isLinkDuplicate
 *
 * @brief       Help determine if the link has already been established.. Defense
 *              against duplicate link frames. This file owns the data structure
 *              so the comparison is done here.
 *
 * input parameters
 * @param   addr       - pointer to address of linker in question
 * @param   remotePort - remote port number provided by linker
 *
 * output parameters
 *
 * @return   Returns pointer to connection entry if the address and remote Port
 *           match an existing entry, otherwise 0.
 */
connInfo_t *nwk_isLinkDuplicate(uint8_t *addr, uint8_t remotePort)
{
#if NUM_CONNECTIONS > 0
  uint8_t       i;
  connInfo_t   *ptr = sPersistInfo.connStruct;

  for (i=0; i<NUM_CONNECTIONS; ++i,++ptr)
  {
    if (CONNSTATE_CONNECTED == ptr->connState)
    {
      if (!(memcmp(ptr->peerAddr, addr, NET_ADDR_SIZE)) &&
          (ptr->portTx == remotePort))
      {
      ptr->idle = 0;
        return ptr;
      }
    }
  }
#endif

  return (connInfo_t *)0;
}

/******************************************************************************
 * @fn          nwk_findAddressMatch
 *
 * @brief       Used to look for a match when a Tx-only device sends to the
 *              Tx-only port. If a match is found the local Rx port will be
 *              substituted for the Tx-only port in the frame so the local
 *              application can receive the payload "normally". Match is based
 *              on source address in frame.
 *
 * input parameters
 * @param   frame    - pointer to frame in question
 *
 * output parameters
 *
 * @return   Returns pointer to the matching connection info structure if
 *           a match is found, otherwise NULL.
 */
connInfo_t *nwk_findAddressMatch(mrfiPacket_t *frame)
{
#if NUM_CONNECTIONS > 0
  uint8_t       i;
  connInfo_t   *ptr = sPersistInfo.connStruct;

  for (i=0; i<NUM_CONNECTIONS; ++i,++ptr)
  {

    if (CONNSTATE_CONNECTED == ptr->connState)
    {
      if (!(memcmp(ptr->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE)))
      {
        return ptr;
      }
    }
  }
#endif

  return (connInfo_t *)0;
}

//added by cbwang
/******************************************************************************
 * @fn          nwk_isLinked
 *
 * @brief       Helper function to look thorugh the list of joined devices to
 *              see if this one is new.
 *
 * input parameters
 * @param   frame   - pointer to received join frame
 *
 * output parameters
 *
 * @return   Returns '1' if the device has already joined, otherwise returns 0.
 */
uint8_t nwk_isLinked(mrfiPacket_t * frame)
{
  uint8_t     i;
  connInfo_t *ptr = sPersistInfo.connStruct;

  for (i=0; i<NUM_CONNECTIONS; ++i,++ptr)
  {
    /* Look for an entry in the link state */
    if ((ptr->connState == CONNSTATE_CONNECTED))
    {
      /* Is this it? */
      if (!(memcmp(&ptr->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE)))
      {
        return 1;
      }
    }
  }

  /* Nothing found... */
  return 0;
}
//end

#if defined(AP_IS_DATA_HUB)
/***************************************************************************************
 * @fn          nwk_saveJoinedDevice
 *
 * @brief       Save the address of a joining device on the Connection Table expecting
 *              a Link frame to follow. Only for when AP is a data hub. We want to
 *              use the space already allocated for a connection able entry instead
 *              of having redundant arrays for alread-joined devices in the data hub
 *              case.
 *
 * input parameters
 * @param   frame  - pointer to frame containing address or joining device.
 *
 * output parameters
 *
 * @return   Returns non-zero if this is a new device and it is saved. Returns
 *           0 if device already there or there is no room in the Connection
 *           Table.
 */
uint8_t nwk_saveJoinedDevice(mrfiPacket_t *frame)
#if 0 //orign version
{
  uint8_t     i;
  connInfo_t *avail = 0;
  connInfo_t *ptr   = sPersistInfo.connStruct;

  for (i=0; i<NUM_CONNECTIONS; ++i, ++ptr)
  {
    if ((ptr->connState == CONNSTATE_CONNECTED) || (ptr->connState == CONNSTATE_JOINED))
    {
      if (!memcmp(ptr->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE))
      {
        return 2;
      }
    }
    else
    {
      avail = ptr;
      break;
    }
  }

  if (!avail)
  {
  	ptr   = sPersistInfo.connStruct;
  	for (i=0; i<NUM_CONNECTIONS; ++i, ++ptr)
	{
		if(++ptr->idle >= CONN_IDEL_MAX)
		{
			avail = ptr;
			break;
		}
	}
	if (!avail)
		return 0;
  }

  avail->connState = CONNSTATE_JOINED;
  avail->idle = 0;
  memcpy(avail->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);

  return 1;
}
#else
{
  uint8_t     i;
  connInfo_t *avail = 0;
  connInfo_t *ptr   = sPersistInfo.connStruct;

  for (i=0; i<NUM_CONNECTIONS; ++i, ++ptr)
  {
    if ((ptr->connState == CONNSTATE_CONNECTED) || (ptr->connState == CONNSTATE_JOINED))
    {
      if (!memcmp(ptr->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE))
      {
        return 0;
      }
    }
    else
    {
      avail = ptr;
    }
  }

  if (!avail)
  {
    return 0;
  }

  avail->connState = CONNSTATE_JOINED;
  memcpy(avail->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);

  return 1;
}
#endif

/***********************************************************************************
 * @fn          nwk_findAlreadyJoined
 *
 * @brief       Used when AP is a data hub to look for an address match in the
 *              Connection table for a device that is already enterd in the joined
 *              state. This means that the Connection Table resource is already
 *              allocated so the link-listen doesn't have to do it again. Match is
 *              based on source address in frame. Thsi shoudl only be called from
 *              the Link-listen context during the link frame reply.
 *
 *              If found the Connection Table entry is initialized as if it were
 *              found using the nwk_getNextConnection() method.
 *
 * input parameters
 * @param   frame    - pointer to frame in question
 *
 * output parameters
 *
 * @return   Returns pointer to Connection Table entry if match is found, otherwise
 *           0. This call will only fail if the Connection Table was full when the
 *           device tried to join initially.
 */
connInfo_t *nwk_findAlreadyJoined(mrfiPacket_t *frame)
{
  uint8_t     i;
  connInfo_t *ptr = sPersistInfo.connStruct;

  for (i=0; i<NUM_CONNECTIONS; ++i,++ptr)
  {
    /* Look for an entry in the JOINED state */
    if (CONNSTATE_JOINED == ptr->connState)
    {
      /* Is this it? */
      if (!(memcmp(&ptr->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE)))
      {
        /* Yes. Initilize tabel entry and return the pointer. */
        initializeConnection(ptr);
        return ptr;
      }
    }
  }

  /* Nothing found... */
  return (connInfo_t *)NULL;
}

/******************************************************************************
 * @fn          nwk_isJoined
 *
 * @brief       Helper function to look thorugh the list of joined devices to
 *              see if this one is new.
 *
 * input parameters
 * @param   frame   - pointer to received join frame
 *
 * output parameters
 *
 * @return   Returns '1' if the device has already joined, otherwise returns 0.
 */
uint8_t nwk_isJoined(mrfiPacket_t * frame)
{
  uint8_t     i;
  connInfo_t *ptr = sPersistInfo.connStruct;

  for (i=0; i<NUM_CONNECTIONS; ++i,++ptr)
  {
    /* Look for an entry in the JOINED state */
    if ((ptr->connState == CONNSTATE_CONNECTED) || (ptr->connState == CONNSTATE_JOINED))
    {
      /* Is this it? */
      if (!(memcmp(&ptr->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE)))
      {
        return 1;
      }
    }
  }

  /* Nothing found... */
  return 0;
}

#endif  /* AP_IS_DATA_HUB */
/******************************************************************************
 * @fn          nwk_checkConnInfo
 *
 * @brief       Do a sanity/validity check on the connection info
 *
 * input parameters
 * @param   ptr     - pointer to a valid connection info structure to validate
 * @param   which   - Tx or Rx port checked
 *
 * output parameters
 *
 * @return   Status of operation.
 */
smplStatus_t nwk_checkConnInfo(connInfo_t *ptr, uint8_t which)
{
  uint8_t  port;

  /* make sure port isn't null and that the entry is active */
  port = (CHK_RX == which) ? ptr->portRx : ptr->portTx;
  if (!port || (CONNSTATE_FREE == ptr->connState))
  {
    return SMPL_BAD_PARAM;
  }

  /* validate port number */
  if (port < PORT_BASE_NUMBER)
  {
    return SMPL_BAD_PARAM;
  }

  return SMPL_SUCCESS;
}

/******************************************************************************
 * @fn          nwk_isConnectionValid
 *
 * @brief       Do a sanity/validity check on the frame target address by
 *              validating frame against connection info
 *
 * input parameters
 * @param   frame   - pointer to frame in question
 *
 * output parameters
 * @param   lid   - link ID of found connection
 *
 * @return   0 if connection specified in frame is not valid, otherwise non-zero.
 */
uint8_t nwk_isConnectionValid(mrfiPacket_t *frame, linkID_t *lid)
{
  uint8_t       i;
  connInfo_t   *ptr  = sPersistInfo.connStruct;
  uint8_t       port = GET_FROM_FRAME(MRFI_P_PAYLOAD(frame), F_PORT_OS);

  for (i=0; i<SYS_NUM_CONNECTIONS; ++i,++ptr)
  {
    if (CONNSTATE_CONNECTED == ptr->connState)
    {
      /* check port first since we're done if the port is the user bcast port. */
	printf_1("ptr->portRx: %d\n",ptr->portRx);
      if (port == ptr->portRx || port == SMPL_PORT_CALL || port == SMPL_PORT_MGMT )
      {
        /* yep...ports match. */
        if ((SMPL_PORT_USER_BCAST == port) || !(memcmp(ptr->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE)))
        {
          /* we're done.*/
          *lid = ptr->thisLinkID;
          return 1;
        }
      }
    }
  }

  /* no matches */
  return 0;
}

/******************************************************************************
 * @fn          nwk_allocateLocalRxPort
 *
 * @brief       Allocate a local port on which to receive frames from a peer.
 *
 *              Allocation differs depending on whether the allocation is for
 *              a link reply frame or a link frame. In the former case we
 *              know the address of the peer so we can ensure allocating a
 *              unique port number for that address. The same port number can be
 *              used mulitple times for distinct peers. Allocations are done from
 *              the bottom of the namespace upward.
 *
 *              If allocation is for a link frame we do not yet know the peer
 *              address so we must ensure the port number is unique now.
 *              Allocations are done from the top of the namespace downward.
 *
 *              The two allocation methods track the extreme values used in each
 *              case to detect overlap, i.e., exhausted namespace. This can only
 *              happen if the number of connections supported is greater than the
 *              total namespace available.
 *
 * input parameters
 * @param   which   - Sending a link frame or a link reply frame
 * @param   newPtr  - pointer to connection info structure to be populated
 *
 * output parameters
 * @param   newPtr->portRx  - element is populated with port number.
 *
 * @return   Non-zero if port number assigned. 0 if no port available.
 */
uint8_t nwk_allocateLocalRxPort(uint8_t which, connInfo_t *newPtr)
{
#if NUM_CONNECTIONS > 0
#ifndef RX_NEVER
  uint8_t     num, i;
  uint8_t     marker[NUM_CONNECTIONS];
  connInfo_t *ptr = sPersistInfo.connStruct;

  memset(&marker, 0x0, sizeof(marker));

  for (i=0; i<NUM_CONNECTIONS; ++i,++ptr)
  {
    /* Mark the port number as used unless it's a statically allocated port */
    if ((ptr != newPtr) && (CONNSTATE_CONNECTED == ptr->connState) && (ptr->portRx <= SMPL_PORT_USER_MAX))
    {
      if (LINK_SEND == which)
      {
        if (ptr->portRx > sPersistInfo.curNextLinkPort)
        {
          marker[SMPL_PORT_USER_MAX - ptr->portRx] = 1;
        }
      }
      else if (!memcmp(ptr->peerAddr, newPtr->peerAddr, NET_ADDR_SIZE))
      {
          marker[ptr->portRx - PORT_BASE_NUMBER] = 1;
      }
    }
  }

  num = 0;
  for (i=0; i<NUM_CONNECTIONS; ++i)
  {
    if (!marker[i])
    {
      if (LINK_REPLY == which)
      {
        num = PORT_BASE_NUMBER + i;
      }
      else
      {
        num = SMPL_PORT_USER_MAX - i;
      }
      break;
    }
  }

  if (LINK_REPLY == which)
  {
    /* if the number we have doesn't overlap the assignment of ports used
     * for sending link frames, use it.
     */
    if (num <= sPersistInfo.curNextLinkPort)
    {
      if (num > sPersistInfo.curMaxReplyPort)
      {
        /* remember maximum port number used */
        sPersistInfo.curMaxReplyPort = num;
      }
    }
    else
    {
      /* the port number we need has already been used in the other context. It may or
       * may not have been used for the same address but we don't bother to check...we
       * just reject the asignment. This is the overly cautious part but is extermely
       * unlikely to ever occur.
       */
      num = 0;
    }
  }
  else
  {
    /* if the number we have doesn't overlap the assignment of ports used
     * for sending link frame replies, use it.
     */
    if (num >= sPersistInfo.curMaxReplyPort)
    {
      if (num == sPersistInfo.curNextLinkPort)
      {
        sPersistInfo.curNextLinkPort--;
      }
    }
    else
    {
      /* the port number we need has already been used in the other context. It may or
       * may not have been used for the same address but we don't bother to check...we
       * just reject the asignment. This is the overly cautious part but is extermely
       * unlikely to ever occur.
       */
      num = 0;
    }
  }

  newPtr->portRx = num;

  return num;
#else
  // vacuously succeeds if device is Tx-only.
  newPtr->portRx = 0;

  return 1;
#endif  // RX_NEVER
#else
  return 0;
#endif  /* NUM_CONNECTIONS > 0 */

}

/*******************************************************************************
 * @fn          nwk_isValidReply
 *
 * @brief       Examine a frame to see if it is a valid reply when compared with
 *              expected parameters.
 *
 * input parameters
 * @param   frame      - pointer to frmae being examined
 * @param   tid        - expected transaction ID in application payload
 * @param   infoOffset - offset to payload information containing reply hint
 * @param   tidOffset  - offset to transaction ID in payload
 *
 * output parameters
 *
 * @return   reply category:
 *               SMPL_NOT_REPLY: not a reply
 *               SMPL_MY_REPLY : a reply that matches input parameters
 *               SMPL_A_REPLY  : a reply but does not match input parameters
 */
uint8_t nwk_isValidReply(mrfiPacket_t *frame, uint8_t tid, uint8_t infoOffset, uint8_t tidOffset)
{
  uint8_t rc = SMPL_NOT_REPLY;

  if ((*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+infoOffset) & NWK_APP_REPLY_BIT))
  {
    if ((*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+tidOffset) == tid) &&
        !memcmp(MRFI_P_DST_ADDR(frame), nwk_getMyAddress(), NET_ADDR_SIZE))
    {
      rc = SMPL_MY_REPLY;
    }
    else
    {
      rc = SMPL_A_REPLY;
    }
  }

  return rc;
}

/******************************************************************************
 * @fn          map_lid2idx
 *
 * @brief       Map link ID to index into connection table.
 *
 * input parameters
 * @param   lid   - Link ID to be matched
 *
 * output parameters
 * @param   idx   - populated with index into connection table
 *
 * @return   Non-zero if Link ID found and output is valid else 0.
 */
static uint8_t map_lid2idx(linkID_t lid, uint8_t *idx)
{
  uint8_t     i;
  connInfo_t *ptr = sPersistInfo.connStruct;

  for (i=0; i<SYS_NUM_CONNECTIONS; ++i, ++ptr)
  {
    if ((CONNSTATE_CONNECTED == ptr->connState) && (ptr->thisLinkID == lid))
    {
      *idx = i;
      return 1;
    }
  }

  return 0;
}
