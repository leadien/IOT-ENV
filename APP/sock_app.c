#include <string.h>
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "lwip/api.h"
#include "lwip/stats.h"
#include "netif/etharp.h"
#include "netif/loopif.h"
#include "netif/dm9kif.h"
#include "arch/sys_arch.h"

#include "lwip\sockets.h"
#include "lwip\inet.h"
#include "ucos_ii.h"
#include "uart.h"
#include "sock_app.h"
#include "dtimer.h"

OS_EVENT *g_SockCtrlQ=NULL,*g_SockMutex=NULL,*g_SendSem,*g_SockResetQ=NULL;
void *SockCtrlMSGQ[SOCK_MSG_NUM],*SockResetMSGQ[SOCK_RESET_MSG_NUM];
uint32_t g_currentSockSendMSG=0;
uint32_t g_TCPPingCount=0;
static int g_sockfd = -1;
static uint8_t g_connect_ok = 0;
static int32_t g_pingtimer = -1;

extern unsigned int g_dbg_cnt;
extern struct netif *dm9k_netif;
void dhcp_init(struct netif *netif);

void WERS_TCPRecevieTask(void *arg);
void WERS_TCPSendPing(void);

void TCP_entry_task(void *arg)
{
	//int sockfd;
	//int len;
	int sockerr;
	int optval=1;
	int connect_cnt;
	INT8U err;
	int sockctrl;
	//int ping_timer;
	//static char receivebuf[64];
	struct sockaddr_in servaddr;
	printf("Enter TCP_entry_task\n");
	g_SockCtrlQ=OSQCreate(SockCtrlMSGQ, SOCK_MSG_NUM);
	g_SockResetQ=OSQCreate(SockResetMSGQ, SOCK_RESET_MSG_NUM);
	g_SockMutex=OSMutexCreate(0, &err);
	g_SendSem=OSSemCreate(0);
	//sockfd=lwip_socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	g_sockfd=lwip_socket(PF_INET, SOCK_STREAM, 0);
	//sockfd=lwip_socket(PF_INET, SOCK_DGRAM, 0);
	//BZERO(servaddr, sizeof(servaddr));
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = dm9k_netif->gw.addr;
	//servaddr.sin_addr.s_addr=inet_addr("192.168.1.110");
	//servaddr.sin_addr.s_addr=inet_addr("192.168.1.108");
	//servaddr.sin_port = htons(8080);
	//servaddr.sin_port = htons(2020);
	//if(dm9k_netif->EMenuCom_Port)
	//	servaddr.sin_port = htons(dm9k_netif->EMenuCom_Port);
	//else
	servaddr.sin_port = htons(10000);
		
	connect_cnt = 0;
	while(1){
		sockerr=lwip_connect(g_sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
		if(sockerr==0)break;
		connect_cnt ++;
		if(connect_cnt>1)
		{
			dhcp_init(dm9k_netif);
			servaddr.sin_addr.s_addr = dm9k_netif->gw.addr;
			connect_cnt = 0;	
		}
		OSTimeDly(OS_TICKS_PER_SEC);
	}
	lwip_setsockopt(g_sockfd, IPPROTO_TCP, TCP_NODELAY, &optval, 4);
	//sys_thread_new(readtask, (void*)sockfd, 0);
	g_connect_ok = 1;
	sys_thread_new(WERS_TCPRecevieTask, (void*)g_sockfd, 0);

	g_pingtimer=CreateTimerEx(sock_ping, 2000, DSTIME_MODE_CYCLE);
	StartTimer(g_pingtimer);
	while(1)
	{
		
		
		//len=lwip_write(sockfd, "Hello lwip socket!\r\n",sizeof("Hello lwip socket!\r\n")-1);
		//sockbuf=(SOCK_MSG_T *)OSQAccept(g_SockSendQ, &err);
		sockctrl=(int)OSQPend(g_SockCtrlQ, 0, &err);
		if(err==OS_ERR_NONE)
		{
			//OSMutexPend(g_SockMutex, 0, &err);
			switch(sockctrl)
			{
			case SOCK_CTRL_RESET:
				OSMutexPend(g_SockMutex, 0, &err);
				//if(len == -1)printf("Send Error\n");
				lwip_close(g_sockfd);
				g_sockfd=lwip_socket(PF_INET, SOCK_STREAM, 0);
				//servaddr.sin_family = AF_UNSPEC;
				//lwip_connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
				//servaddr.sin_family = AF_INET;
				connect_cnt = 0;	
				while(1){
					sockerr=lwip_connect(g_sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
					if(sockerr==0)break;
					OSTimeDly(OS_TICKS_PER_SEC);
					connect_cnt ++;
					if(connect_cnt>30*60)
					{
						dm9k_netif->EMenuCom_Port = 0;
						dhcp_init(dm9k_netif);
						servaddr.sin_addr.s_addr = dm9k_netif->gw.addr;
						if(dm9k_netif->EMenuCom_Port)
							servaddr.sin_port = htons(dm9k_netif->EMenuCom_Port);
						else
							servaddr.sin_port = htons(500);
						connect_cnt = 30*60 - 3;	
					}
				}
				lwip_setsockopt(g_sockfd, IPPROTO_TCP, TCP_NODELAY, &optval, 4);
				g_connect_ok = 1;
				OSMutexPost(g_SockMutex);
				OSQPost(g_SockResetQ,(void *)g_sockfd);
				StartTimer(g_pingtimer);
				break;
			case SOCK_CTRL_CLOSEREQ:
				OSMutexPend(g_SockMutex, 0, &err);
				lwip_closereq(g_sockfd);
				OSMutexPost(g_SockMutex);
				break;
			case SOCK_CTRL_PING:
				if(g_TCPPingCount <3)
				{
					//g_TCPPingCount = 0;	//启用该行关闭复位网络功能
					WERS_TCPSendPing();
					g_TCPPingCount++;
				}
				else
				{
					StopTimer(g_pingtimer);
					sock_closereq();
					g_TCPPingCount = 0;
				}
				break;
			//case SOCK_CTRL_PING_RST:
			//	ResetTimer(ping_timer);
			//	break;
			}
			//OSMutexPost(g_SockMutex);
			//rf_free(sockbuf->data);
		}
	}
}

int sock_reset(void)
{
	INT8U err;
	g_connect_ok = 0;
	if(OS_ERR_NONE == OSQPost(g_SockCtrlQ, SOCK_CTRL_RESET))
		return (int)OSQPend(g_SockResetQ, 0, &err);
	OSQFlush(g_SockCtrlQ);
	if(OS_ERR_NONE == OSQPost(g_SockCtrlQ, SOCK_CTRL_RESET))
		return (int)OSQPend(g_SockResetQ, 0, &err);
	return -1;
}

void sock_closereq(void)
{
	OSQPost(g_SockCtrlQ, (void *)SOCK_CTRL_CLOSEREQ);
}

void sock_ping(void)
{
	OSQPost(g_SockCtrlQ, (void *)SOCK_CTRL_PING);
}

void sock_ping_reset(void)
{
//	OSQPost(g_SockCtrlQ, (void *)SOCK_CTRL_PING_RST);
	ResetTimer(g_pingtimer);
}

void sock_ping_stop(void)
{
	StopTimer(g_pingtimer);
	g_TCPPingCount = 0;
}

uint32_t SendDataToPC(uint8_t *pData, uint32_t len)
{
	INT8U err;
	//SOCK_MSG_T *sockbuf;
	int ret = 0;
	if(pData)
	{
		OSMutexPend(g_SockMutex, 0, &err);
		if(g_connect_ok)
		{
			ret = lwip_send(g_sockfd, pData,len,0);
			if(ret <= 0)
				printf("send error\n");
		}
		OSMutexPost(g_SockMutex);
	}
	return ret;
}

struct netif *loop_netif=NULL;

void
tcpip_init_done(void *arg)
{
  sys_sem_t *sem;
  sem = arg;
  sys_sem_signal(*sem);
}

void dhcp_init(struct netif *netif)
{
	uint32_t idhcpre,icount;
	struct ip_addr ipaddr, netmask, gw;
	IP4_ADDR(&gw, 0,0,0,0);
	IP4_ADDR(&ipaddr, 0,0,0,0);
	IP4_ADDR(&netmask, 0,0,0,0);
	netif_set_addr(dm9k_netif, &ipaddr, &netmask, &gw);

#define TIMES_OF_DHCP	2
	for(idhcpre = 0; idhcpre<TIMES_OF_DHCP; idhcpre++ )//dhcp最多重试4遍
	{
		printf("LWIP:start dhcp request \n");
		dhcp_start(dm9k_netif);//广播dhcp请求
		
		IP4_ADDR(&ipaddr, 0,0,0,0);
		for(icount = 0; (icount < 10) && (ipaddr.addr == 0); icount ++ )
		{
			ipaddr.addr = dm9k_netif->ip_addr.addr;
			OSTimeDly(OS_TICKS_PER_SEC/2); 
		}
		// if failed ipaddr = 0.0.0.0 ;timeout = 10 * 1000 ms
		//等待dhcp是否接受到IP了
		// add dns server ip
		//dns_add(0,&dm9k_netif->dhcp->offered_dns_addr[0]);
		//dns_add(1,&dm9k_netif->dhcp->offered_dns_addr[1]);
		//不需要dns的去掉上面两句
		dhcp_stop(dm9k_netif); //一次dhcp结束

		if (ipaddr.addr != 0)
		break;
	}
	if(idhcpre == TIMES_OF_DHCP)
	{
		IP4_ADDR(&gw, 192,168,1,89);
		IP4_ADDR(&ipaddr, 192,168,1,253);
		IP4_ADDR(&netmask, 255,255,255,0);
		netif_set_addr(dm9k_netif, &ipaddr, &netmask, &gw);
		printf("DHCP fail!\n");
	}
	printf("IP:%d.%d.%d.%d\n",(ipaddr.addr>>0) & 0xff,(ipaddr.addr>>8) & 0xff,(ipaddr.addr>>16) & 0xff,ipaddr.addr>>24);
	icount = dm9k_netif->netmask.addr;
	printf("MASK:%d.%d.%d.%d\n",(icount>>0) & 0xff,(icount>>8) & 0xff,(icount>>16) & 0xff,icount>>24);
	icount = dm9k_netif->gw.addr;
	printf("GW:%d.%d.%d.%d\n",(icount>>0) & 0xff,(icount>>8) & 0xff,(icount>>16) & 0xff,icount>>24);
	icount = dm9k_netif->EMenuCom_Port;
	printf("EMenuCom_Port:%d\n",icount);
}

void lwip_init(void)
{
	struct ip_addr ipaddr, netmask, gw;
	sys_sem_t sem;
//	uint32_t tmp=32;
//	err_t result;

	#if LWIP_STATS
  	stats_init();
	#endif
	sys_init();
	mem_init();
	memp_init();
	pbuf_init();
	netif_init();
	
	sem = sys_sem_new(0);
	tcpip_init(tcpip_init_done, &sem);
	sys_sem_wait(sem);
	sys_sem_free(sem);
	
#if 0  
	//add loop interface
	loop_netif = mem_malloc(sizeof(struct netif));
	IP4_ADDR(&gw, 127,0,0,1);
	IP4_ADDR(&ipaddr, 127,0,0,1);
	IP4_ADDR(&netmask, 255,0,0,0);
	netif_add(loop_netif, &ipaddr, &netmask, &gw, NULL, loopif_init, tcpip_input);
#endif

#if 1
	//add dm9000 interface
  	dm9k_netif = mem_malloc(sizeof(struct netif));
#if LWIP_DHCP
	IP4_ADDR(&gw, 0,0,0,0);
	IP4_ADDR(&ipaddr, 0,0,0,0);
	IP4_ADDR(&netmask, 0,0,0,0);
	netif_set_default(netif_add(dm9k_netif, &ipaddr, &netmask, &gw, NULL, dm9kif_init, tcpip_input));
	dhcp_init(dm9k_netif);
#else
	#if 1
	IP4_ADDR(&ipaddr, 192,168,0,10);
	IP4_ADDR(&netmask, 255,255,255,0);
	IP4_ADDR(&gw, 192,168,0,1);
	Flash_Write(tmp,&ipaddr,sizeof(struct ip_addr));
	tmp+=sizeof(struct ip_addr);
	Flash_Write(tmp,&netmask,sizeof(struct ip_addr));
	tmp+=sizeof(struct ip_addr);
	Flash_Write(tmp,&gw,sizeof(struct ip_addr));
	#else
	Flash_Read(&ipaddr,tmp,sizeof(struct ip_addr));
	tmp+=sizeof(struct ip_addr);
	Flash_Read(&netmask,tmp,sizeof(struct ip_addr));
	tmp+=sizeof(struct ip_addr);
	Flash_Read(&gw,tmp,sizeof(struct ip_addr));
	#endif
	netif_set_default(netif_add(dm9k_netif, &ipaddr, &netmask, &gw, NULL, dm9kif_init, tcpip_input));
	//dm9k_netif=netif_add(&ipaddr, &netmask, &gw, NULL, dm9kif_init, tcpip_input);
	//netif_set_default(dm9k_netif);
#endif

#endif
	sys_thread_new(TCP_entry_task,NULL,0);
}

