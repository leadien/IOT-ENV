#include <string.h>
#include "stm32f10x_flash.h"
#include "flash.h"

#define FLASH_BASE 0x08040000

/* Flash Control Register bits */
#define CR_PG_Set                ((uint32_t)0x00000001)
#define CR_PG_Reset              ((uint32_t)0x00001FFE) 
#define CR_PER_Set               ((uint32_t)0x00000002)
#define CR_PER_Reset             ((uint32_t)0x00001FFD)
#define CR_MER_Set               ((uint32_t)0x00000004)
#define CR_MER_Reset             ((uint32_t)0x00001FFB)
#define CR_OPTPG_Set             ((uint32_t)0x00000010)
#define CR_OPTPG_Reset           ((uint32_t)0x00001FEF)
#define CR_OPTER_Set             ((uint32_t)0x00000020)
#define CR_OPTER_Reset           ((uint32_t)0x00001FDF)
#define CR_STRT_Set              ((uint32_t)0x00000040)
#define CR_LOCK_Set              ((uint32_t)0x00000080)

/* FLASH Keys */
#define RDP_Key                  ((uint16_t)0x00A5)
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)

/* Delay definition */
#define EraseTimeout             ((uint32_t)0x00000FFF)
#define ProgramTimeout           ((uint32_t)0x0000000F)

#define FLASH_PAGE_MASK	(FLASH_PAGE_SIZE-1)

static unsigned char FLASH_SWAP[FLASH_PAGE_SIZE];

static int32_t Flash_ErasePage(uint32_t page)
{
	FLASH_Status status;

	status = FLASH_WaitForLastOperation(FLASH_COMPLETE);

	page &= ~FLASH_PAGE_MASK;
	if(status == FLASH_COMPLETE)
	{ 
		/* if the previous operation is completed, proceed to erase the page */
		FLASH->CR|= CR_PER_Set;
		FLASH->AR = page;
		FLASH->CR|= CR_STRT_Set;
		
		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation(EraseTimeout);
		if(status != FLASH_TIMEOUT)
		{
			/* if the erase operation is completed, disable the PER Bit */
			FLASH->CR &= CR_PER_Reset;
			return 0;
		}
	}
	return -1;
}

static int32_t Flash_WritePage(uint32_t page)
{
	const uint16_t *pt=(uint16_t *)FLASH_SWAP;
	__IO uint16_t *wpt;
	uint32_t i,j;
	FLASH->CR |= CR_PG_Set;
	page &= ~FLASH_PAGE_MASK;
	wpt=(uint16_t *)(page);
	for(i=0;i<FLASH_PAGE_SIZE/2;i++, wpt++, pt++)
	{
		//if(*pt == *wpt || *pt == 0xffff)continue;
		//if(*pt == *wpt)continue;
		//if(*pt == 0xffff)continue;
		j=0;
		while(FLASH_WaitForLastOperation(ProgramTimeout) != FLASH_COMPLETE)
			if(j++>5)return -1;
		*wpt=*pt;
	}
	j=0;
	while(FLASH_WaitForLastOperation(ProgramTimeout) != FLASH_COMPLETE)
		if(j++>5)return -1;
	FLASH->CR &= CR_PG_Reset;
	return 0;
}

#if 0
int fmemcmp(__IO const void *p1,const void * p2,uint32_t size)
{
	uint32_t hsize;
	__IO int16_t *pt1 = (__IO int16_t *)((uint32_t)p1 & ~1);
	int16_t *pt2 = (int16_t *)p2,tmp;
	if(((uint32_t)p1) & 1)
	{
		tmp = (*pt1 >> 8);
		if(tmp != (*pt2 & 0xff))(*pt2 & 0xff)-tmp;
		pt1++;
		pt2 = (int16_t *)((int8_t *)pt2+1);
	}
	hsize = size>>1;
	while(hsize--)
	{
		tmp = *pt1;
		if(*pt1 != *pt2)return *pt2-tmp;
		pt1++;
		pt2++;
	}
	if(size & 1)
	{
		tmp = (*pt1 & 0xff);
		if((tmp != (*pt2 & 0xff)))return ((*pt2 & 0xff) - tmp);
	}
	return 0;
}
#else
int fmemcmp(__IO const void *p1,const void * p2,uint32_t size)
{
	__IO int8_t *pt1 = (__IO int8_t *)((uint32_t)p1 & ~1);
	int8_t *pt2 = (int8_t *)p2,tmp;

	while(size--)
	{
		tmp = *pt1;
		if(tmp != *pt2)return *pt2-tmp;
		pt1++;
		pt2++;
	}
	return 0;
}
#endif

int32_t Flash_Write(const uint32_t offset, const void *buf, uint32_t size)
{
	const uint16_t *pt=buf;
	__IO uint16_t *wpt;
	uint8_t *ptmp;
	uint32_t i,j,page,num,wsize;

	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;

	FLASH->SR = (FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	num = FLASH_PAGE_SIZE-(offset & FLASH_PAGE_MASK);
	ptmp = (uint8_t *)(FLASH_BASE + offset);
	page = ((FLASH_BASE+offset) & ~FLASH_PAGE_MASK);
	if(size <= num)
	{
		wsize = size;
		size = 0;
	}
	else
	{
		wsize = num;
		size -= num;
	}
	if(fmemcmp(ptmp,pt,wsize) != 0)	//如果不相同，重写
	{
		for(i=0;i<wsize;i++)
			if(ptmp[i] != 0xff)break;
		if(i<wsize)		//如果不为全1，导出页面，然后擦除再写
		{
			memcpy(FLASH_SWAP, (void *)page,FLASH_PAGE_SIZE);
			memcpy(FLASH_SWAP+(offset & (FLASH_PAGE_SIZE-1)),buf,wsize);
			Flash_ErasePage(page);
			Flash_WritePage(page);
			pt=(uint16_t *)((char *)pt + wsize);
		}
		else		//如果为全1，直接写
		{
			FLASH->CR |= CR_PG_Set;
			wpt = (uint16_t *)(FLASH_BASE + offset&~1);
			if(offset&1)	//如果写入的首地址为奇数
			{
				j=0;
				while(FLASH_WaitForLastOperation(ProgramTimeout) != FLASH_COMPLETE)
					if(j++>5)return -1;
				*wpt = (*wpt&0xff) | (*pt<<8);
				wpt++;
				wsize--;
				pt = (uint16_t*)((uint8_t*)pt+1);
			}
				
			num = wsize>>1;
			for(i=0;i<num;i++,wpt++,pt++)
			{
				j=0;
				while(FLASH_WaitForLastOperation(ProgramTimeout) != FLASH_COMPLETE)
					if(j++>5)return -1;
				*wpt = *pt;
			}
			if(wsize & 1)
			{
				j=0;
				while(FLASH_WaitForLastOperation(ProgramTimeout) != FLASH_COMPLETE)
					if(j++>5)return -1;
				*wpt = (*wpt&0xff00)|(*pt&0xff);
				pt = (uint16_t *)((char *)pt + 1);
			}
			j=0;
			while(FLASH_WaitForLastOperation(ProgramTimeout) != FLASH_COMPLETE)
				if(j++>5)return -1;
			FLASH->CR &= CR_PG_Reset;
		}
	}

	while(size)
	{
		if(size>=FLASH_PAGE_SIZE)
		{
			wsize = FLASH_PAGE_SIZE;
			size -= FLASH_PAGE_SIZE;
		}
		else
		{
			wsize = size;
			size = 0;
		}
		page += FLASH_PAGE_SIZE;
		wpt = (uint16_t *)page;


		if(fmemcmp((void *)wpt,pt,wsize) != 0)	//如果不相同，重写
		{
			j = wsize>>1;
			for(i=0;i<j;i++)
				if(wpt[i] != 0xffff)break;
			if((i==j) && (wsize&1) && wpt[i]&0xff != 0xff)i=0;
			if(i)	//如果全为1，直接写
			{
				FLASH->CR |= CR_PG_Set;
				//wpt = (uint16_t *)(FLASH_BASE + offset);
				num=j;
				for(i=0;i<num;i++, wpt++, pt++)
				{
					j=0;
					while(FLASH_WaitForLastOperation(ProgramTimeout) != FLASH_COMPLETE)
						if(j++>5)return -1;
					*wpt=*pt;
				}
				if(wsize & 1)
				{
					j=0;
					while(FLASH_WaitForLastOperation(ProgramTimeout) != FLASH_COMPLETE)
						if(j++>5)return -1;
					*wpt = (*wpt&0xff00)|(*pt&0xff);
					pt = (uint16_t *)((char *)pt + 1);
				}
				j=0;
				while(FLASH_WaitForLastOperation(ProgramTimeout) != FLASH_COMPLETE)
					if(j++>5)return -1;
				FLASH->CR &= CR_PG_Reset;
			}
			else
			{
				memcpy(FLASH_SWAP, (void *)page,FLASH_PAGE_SIZE);
				memcpy(FLASH_SWAP, pt,wsize);
				Flash_ErasePage(page);
				Flash_WritePage(page);
				pt=(uint16_t *)((char *)pt + wsize);
			}
		}
		else
		{
			pt=(uint16_t *)((char *)pt + wsize);
		}
	}
	
        /* Disable the PG Bit */
	//FLASH->CR &= CR_PG_Reset;
	FLASH->CR |= CR_LOCK_Set;
	return 0;
}

void * Flash_Read(void *buf, uint32_t offset, uint32_t size)
{
	return memcpy(buf,(void*)(FLASH_BASE+offset),size);	
}

void * Flash_GetAddr(uint32_t offset)
{
	return (void*)(FLASH_BASE+offset);	
}

