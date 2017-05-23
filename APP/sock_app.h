#ifndef __SOCK_APP_H__
#define __SOCK_APP_H__

#define SOCK_MSG_NUM		2
#define SOCK_RESET_MSG_NUM	2

#define SOCK_CTRL_RESET		0
#define SOCK_CTRL_CLOSEREQ	1
#define SOCK_CTRL_PING		2
#define SOCK_CTRL_PING_RST	3

extern uint32_t g_TCPPingCount;

int sock_reset(void);
void sock_closereq(void);
void sock_ping(void);
void sock_ping_stop(void);
void sock_ping_reset(void);
uint32_t SendDataToPC(uint8_t *pData, uint32_t len);
void lwip_init(void);

#endif
