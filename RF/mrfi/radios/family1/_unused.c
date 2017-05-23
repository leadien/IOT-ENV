//***********************************************************************************
//  Code that didn't get used.  Could have value at some point though.
//  DO NOT SHIP THIS FILE!
//***********************************************************************************

//  Useful code orginally intended to ship but looks like it causes more problems
//  that it solves.

/**************************************************************************************************
* @fn          mrfiSpiBurstRegWrite
*
* @brief       Read data from radio receive FIFO.
*
* @param       pData - pointer for storing read data
* @param       len   - length of data in bytes
*
* @return      none
**************************************************************************************************
*/
#if 0  /* code is here as a reference if needed */
void mrfiSpiBurstRegWrite(uint8_t addr, uint8_t * pData, uint8_t len);
void mrfiSpiBurstRegWrite(uint8_t addr, uint8_t * pData, uint8_t len)
{
	mrfiSpiIState_t s;
	
	MRFI_SPI_ENTER_CRITICAL_SECTION(s);
	spiBurstAccess(addr | BURST_BIT, pData, len);
	MRFI_SPI_EXIT_CRITICAL_SECTION(s);
}
#endif

// unused comment related to the above code
/*
*              Note :  This function could be used to access registers in burst mode as well.
*              The proper address byte would, of course, be required.  HOwever, the calling
*              routine would need to disable interrupts (i.e. call from within a critical
*              section).  If the routine were interrupted there is no mechanism to send an
*              updated address byte which is needed to resume the access where it was left.
*/




// RF register settings for working CC2500 configuration (now taken from include file)

#ifndef SMARTRF_SETTINGS_H
#define SMARTRF_SETTING_FSCTRL1    0x09
#define SMARTRF_SETTING_FSCTRL0    0x00
#define SMARTRF_SETTING_FREQ2      0x5D
#define SMARTRF_SETTING_FREQ1      0x93
#define SMARTRF_SETTING_FREQ0      0xB1
#define SMARTRF_SETTING_MDMCFG4    0x2D
#define SMARTRF_SETTING_MDMCFG3    0x3B
#define SMARTRF_SETTING_MDMCFG2    0x77
#define SMARTRF_SETTING_MDMCFG1    0x22
#define SMARTRF_SETTING_MDMCFG0    0xF8
#define SMARTRF_SETTING_DEVIATN    0x01
#define SMARTRF_SETTING_MCSM0      0x18
#define SMARTRF_SETTING_FOCCFG     0x1D
#define SMARTRF_SETTING_BSCFG      0x1C
#define SMARTRF_SETTING_AGCCTRL2   0xC7
#define SMARTRF_SETTING_AGCCTRL1   0x00
#define SMARTRF_SETTING_AGCCTRL0   0xB2
#define SMARTRF_SETTING_FREND1     0xB6
#define SMARTRF_SETTING_FREND0     0x10
#define SMARTRF_SETTING_FSCAL3     0xEA
#define SMARTRF_SETTING_FSCAL2     0x0A
#define SMARTRF_SETTING_FSCAL1     0x00
#define SMARTRF_SETTING_FSCAL0     0x11
#define SMARTRF_SETTING_TEST2      0x88
#define SMARTRF_SETTING_TEST1      0x31
#define SMARTRF_SETTING_TEST0      0x0B
#endif

//-------------------------------------------------------------------------
//
//   Optimization for radio register initialization.
//   Only saves 17 bytes, not worth it.  Code is less robust
//   and error prone.
//
const uint8_t mrfiRadioCfg[] =
{
	/*  IOCFG2   */   MRFI_SETTING_IOCFG2,   
	/*  IOCFG1   */   MRFI_SETTING_IOCFG1,
	/*  IOCFG0   */   MRFI_SETTING_IOCFG0,
	/*  FIFOTHR  */   0x07,
	/*  SYNC1    */   MRFI_SETTING_SYNC1,
	/*  SYNC0    */   MRFI_SETTING_SYNC0,
	/*  PKTLEN   */   MRFI_MAX_FRAME_SIZE + MRFI_RX_METRICS_SIZE,
	/*  PKTCTRL1 */   0000,
	/*  PKTCTRL0 */   0000,
	/*  ADDR     */   0x04,
	/*  CHANNR   */   0x00,
	/*  FSCTRL1  */   SMARTRF_SETTING_FSCTRL1,
	/*  FSCTRL0  */   SMARTRF_SETTING_FSCTRL0,
	
	/*  FREQ2    */   SMARTRF_SETTING_FREQ2,
	/*  FREQ1    */   SMARTRF_SETTING_FREQ1,
	/*  FREQ0    */   SMARTRF_SETTING_FREQ0,
	
	/*  MDMCFG4  */   SMARTRF_SETTING_MDMCFG4,
	/*  MDMCFG3  */   SMARTRF_SETTING_MDMCFG3,
	/*  MDMCFG2  */   SMARTRF_SETTING_MDMCFG2,
	/*  MDMCFG1  */   SMARTRF_SETTING_MDMCFG1,
	/*  MDMCFG0  */   SMARTRF_SETTING_MDMCFG0,
	
	/*  DEVIATN  */   SMARTRF_SETTING_DEVIATN,
	
	/*  MCSM2    */   MRFI_SETTING_MCSM2,
	/*  MCSM1    */   MRFI_SETTING_MCSM1,
	/*  MCSM0    */   MRFI_SETTING_MCSM0,
	
	/*  FOCCFG   */   SMARTRF_SETTING_FOCCFG,
	
	/*  BSCFG    */   SMARTRF_SETTING_BSCFG,
	
	/*  AGCCTRL2 */   SMARTRF_SETTING_AGCCTRL2,
	/*  AGCCTRL1 */   SMARTRF_SETTING_AGCCTRL1,
	/*  AGCCTRL0 */   SMARTRF_SETTING_AGCCTRL0,
	
	/*  WOREVT0  */   0x04,
	/*  WORCTRL  */   0x04,
	
	/*  FREND1  */    0x04,
	/*  FREND0  */    0x04,
	
	/*  FSCAL3  */    SMARTRF_SETTING_FSCAL3,
	/*  FSCAL2  */    SMARTRF_SETTING_FSCAL2,
	/*  FSCAL1  */    SMARTRF_SETTING_FSCAL1,
	/*  FSCAL0  */    SMARTRF_SETTING_FSCAL0
};

const uint8_t mrfiRadioCfgT[] =
{
	SMARTRF_SETTING_TEST2,
		SMARTRF_SETTING_TEST1,
		SMARTRF_SETTING_TEST0
};


for (i=0; i<sizeof(mrfiRadioCfg); i++)
{
    mrfiSpiWriteReg(i, mrfiRadioCfg[i]);
}

for (i=0x29; i<0x2A; i++)
{
    mrfiSpiWriteReg(i, mrfiRadioCfgT[i]);
}

//-------------------------------------------------------------------------

/* radio states from status byte */
#define MRFI_RADIO_STATE_MASK               0x70
#define MRFI_RADIO_STATE_IDLE               0x00
#define MRFI_RADIO_STATE_RX                 0x10
#define MRFI_RADIO_STATE_TX                 0x20
#define MRFI_RADIO_STATE_FSTXON             0x30
#define MRFI_RADIO_STATE_CALIBRATE          0x40
#define MRFI_RADIO_STATE_SETTLING           0x50
#define MRFI_RADIO_STATE_RXFIFO_OVERFLOW    0x60
#define MRFI_RADIO_STATE_TXFIFO_UNDERFLOW   0x70

#define MRFI_GET_RADIO_STATE()          (mrfiSpiCmdStrobe(MRFI_CC2500_SPI_STROBE_SNOP) & MRFI_RADIO_STATE_MASK)



