/** @file
 *
 * Dynamic memory manager
 *
 */

/* 
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
//#define DEBUG_LEVEL	1
#include "printf_level.h"
#include <string.h>
#include "stm32_eval.h"
#include "ucos_ii.h"
#include "def.h"
#include "rf_mem.h"


#define RF_MEM_ASSERT_EN

#ifndef RF_MEM_ASSERT_EN
#define RF_MEM_ASSERT(...)
#else
#define RF_MEM_ASSERT(con,...)	if(!(con))printf(__VA_ARGS__)
#endif

typedef struct rf_mem_s {
  mem_size_t next, prev;
#if RF_MEM_ALIGNMENT == 1
  uint8_t used;
#elif RF_MEM_ALIGNMENT == 2
  uint16_t used;
#elif RF_MEM_ALIGNMENT == 4
  uint32_t used;
#elif RF_MEM_ALIGNMENT == 8
  uint64_t used;
#else
#error "unhandled RF_MEM_ALIGNMENT size"
#endif /* RF_MEM_ALIGNMENT */
}rf_mem_t; 


static rf_mem_t *rf_ram_end;
static uint8_t rf_ram[RF_MEM_SIZE + sizeof(rf_mem_t) + RF_MEM_ALIGNMENT];

#define RF_MIN_SIZE 12
#if 0 /* this one does not align correctly for some, resulting in crashes */
#define SIZEOF_STRUCT_RF_MEM (unsigned int)MEM_ALIGN_SIZE(sizeof(rf_mem_t))
#else
#define SIZEOF_STRUCT_RF_MEM (sizeof(rf_mem_t) + \
                          (((sizeof(rf_mem_t) % RF_MEM_ALIGNMENT) == 0)? 0 : \
                          (4 - (sizeof(rf_mem_t) % RF_MEM_ALIGNMENT))))
#endif

static rf_mem_t *rf_lfree;   /* pointer to the lowest free block */

//static OS_EVENT* rf_mem_mutex;

static void
rf_plug_holes(rf_mem_t *mem)
{
  rf_mem_t *nmem;
  rf_mem_t *pmem;

  RF_MEM_ASSERT((uint8_t *)mem >= rf_ram, "plug_holes: mem >= ram");
  RF_MEM_ASSERT((uint8_t *)mem < (uint8_t *)rf_ram_end, "plug_holes: mem < ram_end");
  RF_MEM_ASSERT(mem->used == 0, "plug_holes: mem->used == 0");
  
  /* plug hole forward */
  RF_MEM_ASSERT(mem->next <= RF_MEM_SIZE, "plug_holes: mem->next <= RF_MEM_SIZE");
  
  nmem = (rf_mem_t *)&rf_ram[mem->next];
  if (mem != nmem && nmem->used == 0 && (uint8_t *)nmem != (uint8_t *)rf_ram_end) {
    if (rf_lfree == nmem) {
      rf_lfree = mem;
    }
    mem->next = nmem->next;
    ((rf_mem_t *)&rf_ram[nmem->next])->prev = (uint8_t *)mem - rf_ram;
  }

  /* plug hole backward */
  pmem = (rf_mem_t *)&rf_ram[mem->prev];
  if (pmem != mem && pmem->used == 0) {
    if (rf_lfree == mem) {
      rf_lfree = pmem;
    }
    pmem->next = mem->next;
    ((rf_mem_t *)&rf_ram[mem->next])->prev = (uint8_t *)pmem - rf_ram;
  }

}
void
rf_mem_init(void)
{
  rf_mem_t *mem;
//  INT8U err;

  memset(rf_ram, 0, RF_MEM_SIZE);
  mem = (rf_mem_t *)rf_ram;
  mem->next = RF_MEM_SIZE;
  mem->prev = 0;
  mem->used = 0;
  rf_ram_end = (rf_mem_t *)&rf_ram[RF_MEM_SIZE];
  rf_ram_end->used = 1;
  rf_ram_end->next = RF_MEM_SIZE;
  rf_ram_end->prev = RF_MEM_SIZE;

//  rf_mem_mutex = OSMutexCreate(0, &err);

  rf_lfree = (rf_mem_t *)rf_ram;

#if RF_MEM_STATS
  lwip_stats.mem.avail = RF_MEM_SIZE;
#endif /* RF_MEM_STATS */
}
void
rf_mem_free(void *rmem)
{
  rf_mem_t *mem;
  #if OS_CRITICAL_METHOD == 3
	OS_CPU_SR  cpu_sr;
  #endif

//  INT8U err;

  if (rmem == NULL) {
    printf_1("rf_mem_free(p == NULL) was called.\n");
    return;
  }
  
//  OSMutexPend(rf_mem_mutex, 0, &err);
  OS_ENTER_CRITICAL();

  RF_MEM_ASSERT((uint8_t *)rmem >= (uint8_t *)rf_ram && (uint8_t *)rmem < (uint8_t *)rf_ram_end,
  					"rf_mem_free: legal memory");
  
  if ((uint8_t *)rmem < (uint8_t *)rf_ram || (uint8_t *)rmem >= (uint8_t *)rf_ram_end) {
    printf_1("rf_mem_free: illegal memory\n");
#ifdef RF_MEM_STATS
    ++lwip_stats.mem.err;
#endif /* RF_MEM_STATS */
//    OSMutexPost(rf_mem_mutex);
	OS_EXIT_CRITICAL();
    return;
  }
  mem = (rf_mem_t *)((uint8_t *)rmem - SIZEOF_STRUCT_RF_MEM);

  RF_MEM_ASSERT(mem->used, "rf_mem_free: mem->used");
  
  mem->used = 0;

  if (mem < rf_lfree) {
    rf_lfree = mem;
  }
  
#ifdef RF_MEM_STATS
  lwip_stats.mem.used -= mem->next - ((u8_t *)mem - rf_ram);
  
#endif /* RF_MEM_STATS */
  rf_plug_holes(mem);
//  OSMutexPost(rf_mem_mutex);
  OS_EXIT_CRITICAL();
}
void *
rf_mem_reallocm(void *rmem, mem_size_t newsize)
{
  void *nmem;
  nmem = rf_mem_malloc(newsize);
  if (nmem == NULL) {
    return rf_mem_realloc(rmem, newsize);
  }
  memcpy(nmem, rmem, newsize);
  rf_mem_free(rmem);
  return nmem;
}

void *
rf_mem_realloc(void *rmem, mem_size_t newsize)
{
  mem_size_t size;
  mem_size_t ptr, ptr2;
  rf_mem_t *mem, *mem2;
//  INT8U err;
  #if OS_CRITICAL_METHOD == 3
	OS_CPU_SR  cpu_sr;
  #endif


  /* Expand the size of the allocated memory region so that we can
     adjust for alignment. */
  if ((newsize % RF_MEM_ALIGNMENT) != 0) {
   newsize += RF_MEM_ALIGNMENT - ((newsize + SIZEOF_STRUCT_RF_MEM) % RF_MEM_ALIGNMENT);
  }
  
  if (newsize > RF_MEM_SIZE) {
    return NULL;
  }
  
//  OSMutexPend(rf_mem_mutex, 0, &err);
  OS_ENTER_CRITICAL();
  
  RF_MEM_ASSERT((uint8_t *)rmem >= (uint8_t *)rf_ram && (uint8_t *)rmem < (uint8_t *)rf_ram_end,
  					"rf_mem_realloc: legal memory");
  
  if ((uint8_t *)rmem < (uint8_t *)rf_ram || (uint8_t *)rmem >= (uint8_t *)rf_ram_end) {
    printf_1("rf_mem_realloc: illegal memory\n");
   OS_EXIT_CRITICAL();
   return rmem;
  }
  mem = (rf_mem_t *)((uint8_t *)rmem - SIZEOF_STRUCT_RF_MEM);

  ptr = (uint8_t *)mem - rf_ram;

  size = mem->next - ptr - SIZEOF_STRUCT_RF_MEM;
#ifdef RF_MEM_STATS
  lwip_stats.mem.used -= (size - newsize);
#endif /* RF_MEM_STATS */
  
  if (newsize + SIZEOF_STRUCT_RF_MEM + RF_MIN_SIZE < size) {
    ptr2 = ptr + SIZEOF_STRUCT_RF_MEM + newsize;
    mem2 = (rf_mem_t *)&rf_ram[ptr2];
    mem2->used = 0;
    mem2->next = mem->next;
    mem2->prev = ptr;
    mem->next = ptr2;
    if (mem2->next != RF_MEM_SIZE) {
      ((rf_mem_t *)&rf_ram[mem2->next])->prev = ptr2;
    }

    rf_plug_holes(mem2);
  }
  //OSMutexPost(rf_mem_mutex);
  OS_EXIT_CRITICAL();
  return rmem;
}
void *
rf_mem_malloc(mem_size_t size)
{
  mem_size_t ptr, ptr2;
  rf_mem_t *mem, *mem2;
//  INT8U err;
  #if OS_CRITICAL_METHOD == 3
	OS_CPU_SR  cpu_sr;
  #endif

  if (size == 0) {
    return NULL;
  }

  /* Expand the size of the allocated memory region so that we can
     adjust for alignment. */
  if ((size % RF_MEM_ALIGNMENT) != 0) {
    size += RF_MEM_ALIGNMENT - ((size + SIZEOF_STRUCT_RF_MEM) % RF_MEM_ALIGNMENT);
  }
  
  if (size > RF_MEM_SIZE) {
    return NULL;
  }
  
//  OSMutexPend(rf_mem_mutex, 0, &err);
  OS_ENTER_CRITICAL();

  for (ptr = (uint8_t *)rf_lfree - rf_ram; ptr < RF_MEM_SIZE; ptr = ((rf_mem_t *)&rf_ram[ptr])->next) {
    mem = (rf_mem_t *)&rf_ram[ptr];
    if (!mem->used &&
       mem->next - (ptr + SIZEOF_STRUCT_RF_MEM) >= size + SIZEOF_STRUCT_RF_MEM) {
      ptr2 = ptr + SIZEOF_STRUCT_RF_MEM + size;
      mem2 = (rf_mem_t *)&rf_ram[ptr2];

      mem2->prev = ptr;      
      mem2->next = mem->next;
      mem->next = ptr2;      
      if (mem2->next != RF_MEM_SIZE) {
        ((rf_mem_t *)&rf_ram[mem2->next])->prev = ptr2;
      }
      
      mem2->used = 0;      
      mem->used = 1;
#ifdef RF_MEM_STATS
      lwip_stats.mem.used += (size + SIZEOF_STRUCT_RF_MEM);
      /*      if (lwip_stats.mem.max < lwip_stats.mem.used) {
        lwip_stats.mem.max = lwip_stats.mem.used;
  }*/
      if (lwip_stats.mem.max < ptr2) {
        lwip_stats.mem.max = ptr2;
      }
#endif /* RF_MEM_STATS */

      if (mem == rf_lfree) {
  /* Find next free block after mem */
        while (rf_lfree->used && rf_lfree != rf_ram_end) {
    rf_lfree = (rf_mem_t *)&rf_ram[rf_lfree->next];
        }
        RF_MEM_ASSERT(!rf_lfree->used, "rf_mem_malloc: !lfree->used");
      }
//      OSMutexPost(rf_mem_mutex);
      OS_EXIT_CRITICAL();
      RF_MEM_ASSERT((mem_ptr_t)mem + SIZEOF_STRUCT_RF_MEM + size <= (mem_ptr_t)rf_ram_end,
	    "rf_mem_malloc: allocated memory not above ram_end.");
      RF_MEM_ASSERT((unsigned long)((uint8_t *)mem + SIZEOF_STRUCT_RF_MEM) % RF_MEM_ALIGNMENT == 0,
	    "rf_mem_malloc: allocated memory properly aligned.");
      return (uint8_t *)mem + SIZEOF_STRUCT_RF_MEM;
    }   
  }
  printf_1("rf_mem_malloc: could not allocate %d bytes\n", (int)size);
#ifdef RF_MEM_STATS
  ++lwip_stats.mem.err;
#endif /* RF_MEM_STATS */  
//  OSMutexPost(rf_mem_mutex);
  OS_EXIT_CRITICAL();
  return NULL;
}

