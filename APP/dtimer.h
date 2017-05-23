#ifndef __DTIMER_H__
#define __DTIMER_H__


void DTimerInit(void);

void uDelay(uint16_t usec);
void mDelay(uint32_t msec);
uint32_t tmrMsTimeGet(void);
uint32_t tmrDateTimeGet(void);
void MsDelay(uint32_t msec);
int32_t CreateTimerEx(void (*pISR)(void),uint32_t interval,int16_t mode);
int32_t CreateTimer(void (*pISR)(void),uint32_t interval);
void ResetTimer(int timerID);
void DeleteTimer(int32_t timerID);
void StopTimer(int32_t timerID);
void StartTimer(int32_t timerID);


/* ��ʱ���ṹ�壬��Ա���������� volatile, ����C�������Ż�ʱ���������� */
typedef struct
{
	volatile uint32_t count;	/* ������ */
	volatile uint8_t flag;		/* ��ʱ�����־  */
}SOFT_TMR;

#define DSTIME_MODE_CYCLE	0
#define DSTIME_MODE_ONCE	1
//void StartTimer(uint8_t _id, uint32_t _period);
//uint8_t CheckTimer(uint8_t _id);
//int32_t GetRunTime(void);

#endif
