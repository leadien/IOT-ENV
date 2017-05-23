#ifndef __FLASH_H__
#define __FLASH_H__

/* Private define ------------------------------------------------------------*/
/* Define the STM32F10x FLASH Page Size depending on the used STM32 device */
#ifdef STM32F10X_LD
  #define FLASH_PAGE_SIZE    (0x400u)
#elif defined STM32F10X_MD
  #define FLASH_PAGE_SIZE    (0x400u)
#elif defined STM32F10X_HD
  #define FLASH_PAGE_SIZE    (0x800u)
#elif defined STM32F10X_CL
  #define FLASH_PAGE_SIZE    (0x800u)  
#endif /* STM32F10X_LD */

#define FLASH_AP_DEVICE_CFG	0
#define FLASH_DM9000_MAC		16

//int32_t Flash_Write(uint32_t offset, const void *buf, uint32_t size);
int32_t Flash_Write(const uint32_t offset, const void *buf, uint32_t size);
void * Flash_Read(void *buf, uint32_t offset, uint32_t size);
void * Flash_GetAddr(uint32_t offset);

#endif
