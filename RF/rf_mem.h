#ifndef __RF_MEM_H__
#define __RF_MEM_H__


#define RF_MEM_ALIGNMENT	4
//#define RF_MEM_SIZE			2048
#define RF_MEM_SIZE			4096

#if RF_MEM_SIZE > 64000l
typedef uint32_t mem_size_t;
#else
typedef uint16_t mem_size_t;
#endif /* MEM_SIZE > 64000 */

void rf_mem_init(void);

void *rf_mem_malloc(mem_size_t size);
void rf_mem_free(void *mem);
void *rf_mem_realloc(void *mem, mem_size_t size);
void *rf_mem_reallocm(void *mem, mem_size_t size);

#ifndef MEM_ALIGN_SIZE
#define MEM_ALIGN_SIZE(size) (((size) + RF_MEM_ALIGNMENT - 1) & ~(RF_MEM_ALIGNMENT-1))
#endif

typedef unsigned int mem_ptr_t;

#ifndef MEM_ALIGN
#define MEM_ALIGN(addr) ((void *)(((mem_ptr_t)(addr) + RF_MEM_ALIGNMENT - 1) & ~(mem_ptr_t)(RF_MEM_ALIGNMENT-1)))
#endif

#endif
