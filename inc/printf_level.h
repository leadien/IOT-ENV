/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2004 by Sunplus Technology Co., Ltd.             *
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
 *  Author: 		                                                      *
 *                                                                        *
 **************************************************************************/
#ifndef _PRINTF_LEVEL_H_
#define _PRINTF_LEVEL_H_

#include <stdio.h>
//#include "customize.h"

//#define FW_RELEASE_CODE	1
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL	0
#endif

//#define DPRINTF Uart_Printf
#define DPRINTF printf

//#define Uart_Printf	printf
void Uart_Printf(const char *fmt,...);

//#define printf(p, ...)
//#define DbgMsdcCtrl(...)
#define DbgMsdcCtrl(...)	DPRINTF(__VA_ARGS__)

#if 0
#define dprintf(...)
#define dprint(str)
#define dprint1(str, a0)
#define dprint2(str, a0, a1)
#define dprint3(str, a0, a1, a2)
#else
#define dprintf(...)	DPRINTF(__VA_ARGS__)
#define dprint(str)	DPRINTF(str)
#define dprint1(str, a0)
#define dprint2(str, a0, a1)
#define dprint3(str, a0, a1, a2)
#endif

#if (DEBUG_LEVEL == 0)
#define printf_0	DPRINTF
#define printf_1(...)
#define printf_2(...)
#define printf_3(...)
#define printf_4(...)

#define dbgPrintf(...)
#define dbgPrintf0(...)
#define dbgPrintf1(...)
#define dbgPrintf2(...)
#define dbgPrintf3(...)
#endif

#if (DEBUG_LEVEL == 1)
#define printf_0	DPRINTF
#define printf_1	DPRINTF
#define printf_2(...)
#define printf_3(...)
#define printf_4(...)

#define dbgPrintf(...)
#define dbgPrintf0(...)		DPRINTF(__VA_ARGS__)
#define dbgPrintf1(...)
#define dbgPrintf2(...)
#define dbgPrintf3(...)
#endif

#if (DEBUG_LEVEL == 2)
#define printf_0	DPRINTF
#define printf_1	DPRINTF
#define printf_2	DPRINTF
#define printf_3(...)
#define printf_4(...)

#define dbgPrintf(...)
#define dbgPrintf0(...)		DPRINTF(__VA_ARGS__)
#define dbgPrintf1(...)		DPRINTF(__VA_ARGS__)
#define dbgPrintf2(...)
#define dbgPrintf3(...)
#endif

#if (DEBUG_LEVEL == 3)
#define printf_0	DPRINTF
#define printf_1	DPRINTF
#define printf_2	DPRINTF
#define printf_3	DPRINTF
#define printf_4(...)
#define dbgPrintf(...)
#define dbgPrintf0(...)		DPRINTF(__VA_ARGS__)
#define dbgPrintf1(...)		DPRINTF(__VA_ARGS__)
#define dbgPrintf2(...)		DPRINTF(__VA_ARGS__)
#define dbgPrintf3(...)
#endif

#if (DEBUG_LEVEL >= 4)
#define printf_0	DPRINTF
#define printf_1	DPRINTF
#define printf_2	DPRINTF
#define printf_3	DPRINTF
#define printf_4	DPRINTF
#define dbgPrintf(...)
#define dbgPrintf0(...)		DPRINTF(__VA_ARGS__)
#define dbgPrintf1(...)		DPRINTF(__VA_ARGS__)
#define dbgPrintf2(...)		DPRINTF(__VA_ARGS__)
#define dbgPrintf3(...)		DPRINTF(__VA_ARGS__)
#endif

#ifdef _DEBUG_
#define ERR	"ERROR:"
#define WARN0	"WARN0:"
#define WARN1	"WARN1:"
#define WARN2	"WARN2:"
#define WARN3	"WARN3:"
#define DPRINTL(level)	DPRINTF(level"in file %s,in line %d",__FILE__,__LINE__)
#define DPRINT()			DPRINTF("in file %s,in line %d",__FILE__,__LINE__)
#endif
#define ASSERT(cond)	do{\
		if(cond){\
			printf("ASSERT:in file %s,in line %d",__FILE__,__LINE__);\
		}\
	}while(0)
#endif
