#ifndef __DBGUART_H__
#define __DBGUART_H__

void init_dbguart(void);
#ifdef WERS_EXTENDER_DEVICE
void init_485_uart(void);
#endif
void Dump(void *buf,uint32_t len);

#endif
