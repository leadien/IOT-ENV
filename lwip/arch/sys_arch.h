#ifndef __SYS_ARCH_H__
#define __SYS_ARCH_H__

#include    "os_cpu.h"
#include    "os_cfg.h"
#include    "ucos_ii.h"


//#define LWIP_STK_SIZE      256
#define LWIP_STK_SIZE      192

//yangye 2003-1-27
//defines for sys_thread_new() & sys_arch_timeouts()
//max number of lwip tasks
#define LWIP_TASK_MAX    4
//first prio of lwip tasks
//#define LWIP_START_PRIO  5  //so priority of lwip tasks is from 5-9 
#define LWIP_START_PRIO  15  //so priority of lwip tasks is from 5-9 


#define SYS_MBOX_NULL   (void*)0
#define SYS_SEM_NULL    (void*)0
//#define MAX_QUEUES        16
//#define MAX_QUEUE_ENTRIES 16
#define MAX_QUEUES        8
#define MAX_QUEUE_ENTRIES 32
//#define MAX_QUEUES        8
//#define MAX_QUEUE_ENTRIES 8

//added by shichu
#define SYS_NO_ERR	OS_NO_ERR
#define SYS_Q_FULL	OS_Q_FULL
#define SYS_Q_EMPTY	OS_Q_EMPTY
//add end

typedef struct {
    OS_EVENT*   pQ;
    void*       pvQEntries[MAX_QUEUE_ENTRIES];
} TQ_DESCR, *PQ_DESCR;
    
typedef OS_EVENT* sys_sem_t;
typedef PQ_DESCR  sys_mbox_t;
typedef INT8U     sys_thread_t;

#endif
