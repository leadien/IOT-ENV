/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2002 by Sunplus Technology Co., Ltd.             *
 *                                                                        *
 *  This software is copyrighted by and is the property of Sunplus        *
 *  Technology Co., Ltd. All rights are reserved by Sunplus Technology    *
 *  Co., Ltd. This software may only be used in accordance with the       *
 *  corresponding license agreement. Any unauthorized use, duplication,   *
 *  distribution, or disclosure of this software is expressly forbidden.  *
 *                                                                        *
 *  This Copyright notice MUST not be removed or modified without prior   *
 *  written consent of Sunplus Technology Co., Ltd.                       *
 *                                                                        *
 *  Sunplus Technology Co., Ltd. reserves the right to modify this        *
 *  software without notice.                                              *
 *                                                                        *
 *  Sunplus Technology Co., Ltd.                                          *
 *  19, Innovation First Road, Science-Based Industrial Park,             *
 *  Hsin-Chu, Taiwan, R.O.C.                                              *
 *                                                                        *
 *  Author: Timothy Wu                                                    *
 *                                                                        *
 **************************************************************************/
#ifndef _OS_API_H_
#define _OS_API_H_

#include "os_cpu.h"
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/

//#define OS_MAX_TASKS            64                  /* max no. of tasks   */
//#define OS_MAX_EVENTS           64                  /* max no. of events  */
//#define OS_MAX_QS               64                  /* max no. of queues  */
//#define OS_MAX_FLAGS            64                  /* max no. of flags   */ /*2006/5/22 for event flag*/
#define OS_MAX_NR_MBLK          128                 /* max no. of mem blk */
#define OS_LO_PRIO              (OS_MAX_TASKS - 1)  /* lowest priority    */
#define OS_TASKS_BITMAP_SIZE    (OS_MAX_TASKS / 8)


/* Task priorities */
/* T.B.D.*/
//#define OS_PRIO_SELF            0xff                /* priority of caller */

#define OS_PRIO_OTGDET          36
#define OS_PRIO_OTGTSK          37
#define OS_PRIO_OTGMSDC         38

#define OS_PRIO_DPS             45
#define OS_PRIO_USB_EP0         47
#define OS_PRIO_USB_BULK_CTRL   49
#define OS_PRIO_USB_OTHER		53

#if 0
#define OS_PRIO_UI              16

#define OS_PRIO_CMD             20
#define OS_PRIO_CARD            21

#define OS_PRIO_AAA             35
#define OS_PRIO_OTGDET          36
#define OS_PRIO_OTGTSK          37
#define OS_PRIO_OTGMSDC         38

#define OS_PRIO_ASF_CLIP        40
#define OS_PRIO_ASF_PLAY        41
#define OS_PRIO_AUDIO           42

#define OS_PRIO_DPS             45
/*#define OS_PRIO_USB_EP0_MSG     46*/
#define OS_PRIO_USB_EP0         47
/*#define OS_PRIO_USB_BULK_MSG    48*/
#define OS_PRIO_USB_BULK_CTRL   49
/*#define OS_PRIO_USB_PLUG		50*/
/*#define OS_PRIO_USB_RESET       51*/
/*#define OS_PRIO_USB_DPC         52*/
#define OS_PRIO_USB_OTHER		53

#define OS_PRIO_MP3_PLAY        55
#define OS_PRIO_MP3_RECORD      56

#define OS_PRIO_STILL_PB        58
#define OS_PRIO_STILL_SNAP      59
#define OS_PRIO_VFS             60
#endif

#define OS_NO_WAIT              0xffff
#define OS_WAIT_FOREVER         0x0000


#endif /* _OS_API_H_ */

