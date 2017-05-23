#include <stdio.h>
//#include "stm32_eval.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "dtimer.h"
//#include "def.h"
#include "bsp.h"
#include "cc1100.h"
#include "ucos_ii.h"

#if 0

#if 1
#define OSTimeDly(ticks)
#define tmrMsTimeGet()	200
#else
//extern void OSTimeDly(uint32_t ticks);
void OSTimeDly(INT16U ticks);
#endif

#endif

#define GDO0 GPIO_ReadInputDataBit(MRFI_GDO_PORT,MRFI_GDO0_PIN)
#define GDO2 GPIO_ReadInputDataBit(MRFI_GDO_PORT,MRFI_GDO2_PIN)

#define	CSn_LOW        SPI_CC1100_CS_LOW()
#define	CSn_HIGH       SPI_CC1100_CS_HIGH()

void SPI_CC1100_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
//    GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
	
	/* Enable SPI and GPIO clocks */
	//RCC_APB1PeriphClockCmd(SPI_CC1100_CLK, ENABLE);
	//RCC_APB2PeriphClockCmd(SPI_CC1100_GPIO_CLK | SPI_CC1100_CS_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(SPI_CC1100_CLK | SPI_CC1100_GPIO_CLK | SPI_CC1100_CS_GPIO_CLK, ENABLE);
	
	/* Configure SPI pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = SPI_CC1100_PIN_SCK | SPI_CC1100_PIN_MISO | SPI_CC1100_PIN_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_CC1100_GPIO, &GPIO_InitStructure);
	
	/* Configure I/O for cc1100 Chip select */
	GPIO_InitStructure.GPIO_Pin = SPI_CC1100_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_CC1100_CS_GPIO, &GPIO_InitStructure);
	
	/* Deselect the FLASH: Chip Select high */
	SPI_CC1100_CS_HIGH();
	
	/* SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	//SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	//SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;	//1.125M
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;	//2.25M
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	//4.5M
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//9M

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_CC1100, &SPI_InitStructure);
	
	/* Enable the SPI  */
	SPI_Cmd(SPI_CC1100, ENABLE);
	
}
	
uint8_t SPI_CC1100_SendByte(uint8_t byte)
{
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI_CC1100, SPI_I2S_FLAG_TXE) == RESET);
	
	/* Send byte through the SPI1 peripheral */
	SPI_I2S_SendData(SPI_CC1100, byte);
	
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI_CC1100, SPI_I2S_FLAG_RXNE) == RESET);
	
	/* Return the byte read from the SPI bus */
	//return SPI_I2S_ReceiveData(SPI_CC1100);
	return SPI_CC1100->DR;
}

#if 0
//UBYTE txBuffer[] = {22,0,0x80,0x01,0xFD,0x01,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A};
uint8_t txBuffer[] = {68,0,0x80,0x01,0xFC,0x01,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,
                              0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,
                              0x28,0x29,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x40,0x41,0x41,0x42,0x43,
                              0x44,0x45,0x46,0x47,0x48,0x49,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59};

#else
uint8_t txBuffer[] = {7,0,0x80,0x01,0xFA,0x01,0x00,0x01};
#endif
uint8_t menuBuffer[] = {13,0,0x80,0x01,   0x00,0x01,  0x01,  0x00,0x01,   0x00,0x01,   0x00,0x0A,0x02};
uint8_t rxBuffer[255];

TX_DATA txData;
RX_DATA  rxData;

void CC1100Init(void);
void PacketReceive(void);
void PacketSend(void);
bool SendData(uint8_t *pBuffer, uint8_t length);
bool ReceiveData(uint8_t *pBuffer);
void CC1100Test(void);
	
extern void ConfigCC1100(void);
void CC1100Init(void)
{
	SPI_CC1100_Init();
//	RESET_CCxxx0();
	//halRfWriteRfSettings();
//	ConfigCC1100();
	//sio_printf1("Chip ID\n");
	printf("Chip ID = %d\r\n",halSpiReadStatus(0x31));
	halSpiWriteReg(CCxxx0_FIFOTHR, 0x0E);
	halSpiWriteReg(CCxxx0_PATABLE, 0xC0);
#if 0
	while(1)
	{
		UINT8 i,j,temp;//,n;
		rSPCON1|=(1<<4);//dis-SCK 

		rSPTDAT1=0xFF;
		while(!(rSPSTA1&0x1));   //Check Rx ready state  
		temp=rSPRDAT1;

		rSPCON1&=~(1<<4);//dis-SCK
	}
#endif
}

void CC1100Test(void)
{
	CC1100Init();
	if (SendData(menuBuffer,sizeof(menuBuffer)))
	printf("OK1\r\n");
	//OSTimeDly(500);
}

bool ReceiveData(uint8_t *pBuffer)
{
      UINT8 i;
	
	for (i = 0; i < 255; i++)
	   	pBuffer[i] = 0;

      rxData.bytesLeft = 0;          		        
      rxData.packetsReceived = 0;   		 
      rxData.pBufferIndex = pBuffer;        		 
      rxData.lengthByte = 0;           		 
      rxData.packetReceivedFlag = FALSE;   	 
      rxData.crcOK = FALSE;                			 

      PacketReceive();
      if (rxData.packetReceivedFlag) 
      {
            rxData.packetReceivedFlag = FALSE;
            rxData.pBufferIndex = rxBuffer; 
            if (rxData.crcOK) 
                 return TRUE;
      }
	return FALSE;
}

bool SendData(uint8_t *pBuffer, uint8_t length)
{
      txData.packetLength = length;	
      txData.bytesLeft = 0;                        
      txData.iterations = 0;                         
      txData.pBufferIndex = pBuffer;                  
      txData.writeRemainingDataFlag = FALSE;  
      txData.packetSentFlag = FALSE;                
    
      PacketSend();
      if (txData.packetSentFlag)
      {
            txData.packetSentFlag = FALSE;
            return TRUE;
      }
	return FALSE;
}
//-------------------------------------------------------------------------------------------------------
//  void PacketsReceive(void)
//  
//  DESCRIPTION: 
//      This function can be used to receive a packet of variable packet length (first byte in the packet
//      must be the length byte). The packet length should not exceed the RX FIFO size.
//      To use this function, GD00 must be configured to be asserted when sync word is sent and 
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG0, 0x06);
//      Also, APPEND_STATUS in the PKTCTRL1 register must be enabled.
//      The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//      for it to be cleared.
//-------------------------------------------------------------------------------------------------------
void PacketReceive(void)
{
	UINT32 stime;
	
	// Associated to the RX FIFO: Asserts when RX FIFO is filled above RXFIFO_THR.
	// De-asserts when RX FIFO is drained below RXFIFO_THR.
	halSpiWriteReg(CCxxx0_IOCFG2, 0x00);
	halSpiStrobe(CCxxx0_SRX);
	
	stime = tmrMsTimeGet();
	
	while (!GDO0)
	{
		if((tmrMsTimeGet()-stime)>100)/*timeout*/
		{
			rxData.packetReceivedFlag = FALSE;
			return;
		}
	}
	// After the sync word is received one needs to wait some time before there will be any data 
	// in the FIFO.In addition, the FIFO should not be emptied 
	// (See the CC1100 or 2500 Errata Note) before the whole packet has been received.
	if (PACKET_INT) 
	{     // If this pin has gone low again, one can assume that the lenght byte was 0 (nothing was put in the RX FIFO)
		OSTimeDly(50); // Allow for 2 bytes to be put in the FIFO (2*8*(1/250000)) = 64 us
		rxData.lengthByte = halSpiReadReg(CCxxx0_RXFIFO);
		rxData.pBufferIndex[0] = rxData.lengthByte;
		rxData.bytesLeft = rxData.lengthByte + 2;
		rxData.pBufferIndex++;
		if (rxData.bytesLeft < FIFO_SIZE)
		rxData.syncOrEndOfPacket = END_OF_PACKET;
		else
		{
			while (1) 
			{
				if (GDO2)
				{ 
					// Do not empty the FIFO (See the CC1100 or 2500 Errata Note)
					halSpiReadBurstReg(CCxxx0_RXFIFO, rxData.pBufferIndex, (BYTES_IN_RX_FIFO - 1));
					rxData.bytesLeft -= (BYTES_IN_RX_FIFO - 1);    
					rxData.pBufferIndex += (BYTES_IN_RX_FIFO - 1);
					if (rxData.bytesLeft <= (BYTES_IN_RX_FIFO - 1))
					{
						while (GDO0);
						break;
					}
				}
			}
		}
	}
	// End of Packet
	halSpiReadBurstReg(CCxxx0_RXFIFO, rxData.pBufferIndex, rxData.bytesLeft); 
	//halSpiStrobe(CCxxx0_SFRX);   
	rxData.syncOrEndOfPacket = SYNC;
	rxData.packetReceivedFlag = END_OF_PACKET;
	//rxData.crcOK = ((rxBuffer[rxData.lengthByte + 2]) & CRC_OK);
	rxData.crcOK = (((rxBuffer[rxData.lengthByte + 2]) & CRC_OK)?TRUE:FALSE);
}

//-------------------------------------------------------------------------------------------------------
//  void PacketSend(void)
//  
//  DESCRIPTION: 
//      This function can be used to transmit a packet with packet length up to 255 bytes.
//      To use this function, GD00 must be configured to be asserted when sync word is sent and 
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG0, 0x06);
//      The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//      for it to be cleared.  
//-------------------------------------------------------------------------------------------------------
void PacketSend(void)
{
	// Associated to the TX FIFO: Asserts when the TX FIFO is filled above TXFIFO_THR.
	// De-asserts when the TX FIFO is below TXFIFO_THR.
	halSpiWriteReg(CCxxx0_IOCFG2, 0x02);
	// The entire packet can be written at once
	if ((txData.packetLength - 1)  < FIFO_SIZE)
	{
		halSpiWriteBurstReg(CCxxx0_TXFIFO, txData.pBufferIndex, txData.packetLength);
		halSpiStrobe(CCxxx0_STX);
	} 
	else
	{      // The TX FIFO needs to be re-filled several times
		halSpiWriteBurstReg(CCxxx0_TXFIFO, txData.pBufferIndex, FIFO_SIZE); // Fill up the TX FIFO
		halSpiStrobe(CCxxx0_STX);
		txData.bytesLeft = txData.packetLength - FIFO_SIZE;
		txData.pBufferIndex = txData.pBufferIndex + FIFO_SIZE;
		txData.iterations = (txData.bytesLeft / AVAILABLE_BYTES_IN_TX_FIFO);
		if (!txData.iterations)
		txData.writeRemainingDataFlag = TRUE; 
		while (1)
		{
			if (!GDO2)
			{
				if (txData.writeRemainingDataFlag)
				{      // Less than 60 bytes to write to the TX FIFO
					halSpiWriteBurstReg(CCxxx0_TXFIFO, txData.pBufferIndex, txData.bytesLeft);
					while (GDO0);
					break;
				}
				else
				{
					halSpiWriteBurstReg(CCxxx0_TXFIFO, txData.pBufferIndex, AVAILABLE_BYTES_IN_TX_FIFO);
					txData.pBufferIndex += AVAILABLE_BYTES_IN_TX_FIFO;
					txData.bytesLeft -= AVAILABLE_BYTES_IN_TX_FIFO;
					if (!(--txData.iterations))
						txData.writeRemainingDataFlag = TRUE;
				}
			}
		}
	}
	txData.writeRemainingDataFlag = FALSE;
	txData.packetSentFlag = TRUE;
}
	
	
//-------------------------------------------------------------------------------------------------------
//  UBYTE SPI_RxTxByte(UBYTE data)
//
//  DESCRIPTION:
//      This function simulates  SPI access timescale.
//
//  ARGUMENTS:
//      UBYTE data
//          Value to be written to the slave device.
//
//  RETURN VALUE:
//      UBYTE
//          Value of the accessed slave device.
//-------------------------------------------------------------------------------------------------------
#ifdef RUN_IN_INTER_MEM
#define SPI_DELAY()	__NOP();__NOP();__NOP();__NOP();__NOP()
#endif
void spi_delay(uint32_t i)
{
	while(i--);
}

uint8_t SPI_RxTxByte(uint8_t data)
{
	uint8_t tmp;
	/* Loop while DR register in not emplty */
	//while (SPI_I2S_GetFlagStatus(SPI_CC1100, SPI_I2S_FLAG_TXE) == RESET);
	while (!(SPI_CC1100->SR & SPI_I2S_FLAG_TXE));
	
	/* Send byte through the SPI1 peripheral */
	//SPI_I2S_SendData(SPI_CC1100, data);
	SPI_CC1100->DR= data;
	
	/* Wait to receive a byte */
	//while (SPI_I2S_GetFlagStatus(SPI_CC1100, SPI_I2S_FLAG_RXNE) == RESET);
	while (!(SPI_CC1100->SR & SPI_I2S_FLAG_RXNE));
	tmp=SPI_CC1100->DR;

//#ifdef WERS_EXTENDER_DEVICE
#if 0
#ifdef RUN_IN_INTER_MEM
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	SPI_DELAY();
	//SPI_DELAY();
	__NOP();
	__NOP();
#endif
#else
#ifdef RUN_IN_INTER_MEM
//	spi_delay(72+72+5);
#endif

#endif
	
	/* Return the byte read from the SPI bus */
	//return SPI_I2S_ReceiveData(SPI_CC1100);
	return tmp;
}
	
//-------------------------------------------------------------------------------------------------------
//  UBYTE halSpiReadStatus(UBYTE addr)
//
//  DESCRIPTION:
//      This function reads a CCxxx0 status register.
//
//  ARGUMENTS:
//      UBYTE addr
//          Address of the CCxxx0 status register to be accessed.
//
//  RETURN VALUE:
//      UBYTE
//          Value of the accessed CCxxx0 status register.
//-------------------------------------------------------------------------------------------------------
uint8_t halSpiReadStatus(uint8_t addr)
{
	UINT8 x;
	
	CSn_LOW;
	//while (SO);
	SPI_RxTxByte(addr | READ_BURST);
	x = SPI_RxTxByte(0);
	CSn_HIGH;
	return x;
}

//-------------------------------------------------------------------------------------------------------
//  UBYTE halSpiReadReg(UBYTE addr)
//
//  DESCRIPTION:
//      This function gets the value of a single specified CCxxx0 register.
//
//  ARGUMENTS:
//      UBYTE addr
//          Address of the CCxxx0 register to be accessed.
//
//  RETURN VALUE:
//      UBYTE
//          Value of the accessed CCxxx0 register.
//-------------------------------------------------------------------------------------------------------
uint8_t halSpiReadReg(uint8_t addr)
{
   	 UINT8 x;
	
    	CSn_LOW;
    	//while (SO);
    	SPI_RxTxByte(addr | READ_SINGLE);
    	x = SPI_RxTxByte(0);
    	CSn_HIGH;
    	return x;
}

//-------------------------------------------------------------------------------------------------------
//  void halSpiWriteReg(UBYTE addr, UBYTE value)
//
//  DESCRIPTION:
//      Function for writing to a single CCxxx0 register
//
//  ARGUMENTS:
//      UBYTE addr
//          Address of a specific CCxxx0 register to accessed.
//      UBYTE value
//          Value to be written to the specified CCxxx0 register.
//-------------------------------------------------------------------------------------------------------
void halSpiWriteReg(uint8_t addr, uint8_t value) 
{
    	CSn_LOW;
    	//while (SO); 
    	SPI_RxTxByte(addr);
    	SPI_RxTxByte(value);
    	CSn_HIGH;
}

//-------------------------------------------------------------------------------------------------------
//  void halSpiReadBurstReg(UBYTE addr, UBYTE *buffer, UBYTE count)		
//
//  DESCRIPTION:
//      This function reads multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      UBYTE addr
//          Address of the first CCxxx0 register to be accessed.
//      UBYTE *buffer
//          Pointer to a byte array which stores the values read from a
//          corresponding range of CCxxx0 registers.
//      UBYTE count
//          Number of bytes to be written to the subsequent CCxxx0 registers.
//-------------------------------------------------------------------------------------------------------
void halSpiReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)			  
{
	UINT8 i; 					
			                           
	CSn_LOW;
	//while (SO); 
	SPI_RxTxByte(addr | READ_BURST);
	for (i = 0; i < count; i++)
	     buffer[i] = SPI_RxTxByte(0);
	CSn_HIGH;
}

//-------------------------------------------------------------------------------------------------------
//  void halSpiWriteBurstReg(UBYTE addr, UBYTE *buffer, UBYTE count)
//
//  DESCRIPTION:
//      This function writes to multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      UBYTE addr
//          Address of the first CCxxx0 register to be accessed.
//      UBYTE *buffer
//          Array of bytes to be written into a corresponding range of
//          CCxx00 registers, starting by the address specified in _addr_.
//      UBYTE count
//          Number of bytes to be written to the subsequent CCxxx0 registers.   
//-------------------------------------------------------------------------------------------------------
void halSpiWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count) 
{
    	UINT8 i;                    
	
    	CSn_LOW;             
    	//while (SO); 
    	SPI_RxTxByte(addr | WRITE_BURST);
    	for (i = 0; i < count; i++) 
		SPI_RxTxByte(buffer[i]);
 
   	CSn_HIGH;
}

//-------------------------------------------------------------------------------------------------------
//  void halSpiStrobe(UBYTE strobe)
//
//  DESCRIPTION:
//      Function for writing a strobe command to the CCxxx0
//
//  ARGUMENTS:
//      UBYTE strobe
//          Strobe command
//-------------------------------------------------------------------------------------------------------
void halSpiStrobe(uint8_t strobe) 
{
    	CSn_LOW;
    	while (GDO0);  
    	SPI_RxTxByte(strobe);
    	CSn_HIGH;
}

//-------------------------------------------------------------------------------------------------------
//  void RESET_CCxxx0(void)
//
//  DESCRIPTION:
//      Reset the CCxxx0 and wait for it to be ready
//
//  ARGUMENTS:
//      
//-------------------------------------------------------------------------------------------------------
void RESET_CCxxx0(void)
{
    CSn_LOW;
    while (GDO0);
    SPI_RxTxByte(CCxxx0_SRES);
   	CSn_HIGH;
}

//-------------------------------------------------------------------------------------------------------
//  void RfWriteRfSettings(void)
//
//  DESCRIPTION:
//      This function is used to configure the CC1100 based on a given rf setting
//
//  ARGUMENTS:
//      RF_SETTINGS *pRfSettings
//          Pointer to a struct containing rf register settings
//-------------------------------------------------------------------------------------------------------
void halRfWriteRfSettings(void) 
{
    // Chipcon
    // Product = CC1100
    // Chip version = F   (VERSION = 0x03)
    // Crystal accuracy = 10 ppm
    // X-tal frequency = 26 MHz
    // RF output power = 0 dBm
    // RX filterbandwidth = 541.666667 kHz
    // Phase = 0
    // Datarate = 249.938965 kBaud
    // Modulation = (7) MSK
    // Manchester enable = (0) Manchester disabled
    // RF Frequency = 432.999817 MHz
    // Channel spacing = 199.951172 kHz
    // Channel number = 0
    // Optimization = Sensitivity
    // Sync mode = (3) 30/32 sync word bits detected
    // Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
    // CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
    // Forward Error Correction = (0) FEC disabled
    // Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
    // Packetlength = 255
    // Preamble count = (2)  4 bytes
    // Append status = 1
    // Address check = (0) No address check
    // FIFO autoflush = 0
    // Device address = 0
    // GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
    // GDO2 signal selection = (41) CHIP_RDY
 
	#if 0  // 250Kbps
    halSpiWriteReg(CCxxx0_FSCTRL1,  0x0A); // FSCTRL1   Frequency synthesizer control.
    halSpiWriteReg(CCxxx0_FSCTRL0,  0x00); // FSCTRL0   Frequency synthesizer control.
    halSpiWriteReg(CCxxx0_FREQ2,    0x10); // FREQ2     Frequency control word, high byte.
    halSpiWriteReg(CCxxx0_FREQ1,    0xA7); // FREQ1     Frequency control word, middle byte.
    halSpiWriteReg(CCxxx0_FREQ0,    0x62); // FREQ0     Frequency control word, low byte.
    halSpiWriteReg(CCxxx0_MDMCFG4,  0x2D);//2   // MDMCFG4   Modem configuration.
    halSpiWriteReg(CCxxx0_MDMCFG3,  0x3B);//3   // MDMCFG3   Modem configuration.
    halSpiWriteReg(CCxxx0_MDMCFG2,  0x73); // MDMCFG2   Modem configuration.
    halSpiWriteReg(CCxxx0_MDMCFG1,  0x22); // MDMCFG1   Modem configuration.
    halSpiWriteReg(CCxxx0_MDMCFG0,  0xF8); // MDMCFG0   Modem configuration.
    halSpiWriteReg(CCxxx0_CHANNR,   0x00); // CHANNR    Channel number.
    halSpiWriteReg(CCxxx0_DEVIATN,  0x00); // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    halSpiWriteReg(CCxxx0_FREND1,   0xB6); // FREND1    Front end RX configuration.
    halSpiWriteReg(CCxxx0_FREND0,   0x10); // FREND0    Front end RX configuration.
    halSpiWriteReg(CCxxx0_MCSM0 ,   0x18); // MCSM0     Main Radio Control State Machine configuration.
    halSpiWriteReg(CCxxx0_FOCCFG,   0x1D);//1  // FOCCFG    Frequency Offset Compensation Configuration.
    halSpiWriteReg(CCxxx0_BSCFG,    0x1C);//1   // BSCFG     Bit synchronization Configuration.
    halSpiWriteReg(CCxxx0_AGCCTRL2, 0xC7);// AGCCTRL2  AGC control.
    halSpiWriteReg(CCxxx0_AGCCTRL1, 0x00); // AGCCTRL1  AGC control.
    halSpiWriteReg(CCxxx0_AGCCTRL0, 0xB0); // AGCCTRL0  AGC control.
    halSpiWriteReg(CCxxx0_FSCAL3,   0xEA);  // FSCAL3    Frequency synthesizer calibration.
    halSpiWriteReg(CCxxx0_FSCAL2,   0x2A);//2   // FSCAL2    Frequency synthesizer calibration.
    halSpiWriteReg(CCxxx0_FSCAL1,   0x00); // FSCAL1    Frequency synthesizer calibration.
    halSpiWriteReg(CCxxx0_FSCAL0,   0x1F);//1// FSCAL0    Frequency synthesizer calibration.
    halSpiWriteReg(CCxxx0_FSTEST,   0x59); // FSTEST    Frequency synthesizer calibration.
    halSpiWriteReg(CCxxx0_TEST2,    0x88); // TEST2     Various test settings.
    halSpiWriteReg(CCxxx0_TEST1,    0x31); // TEST1     Various test settings.
    halSpiWriteReg(CCxxx0_TEST0,    0x09); // TEST0     Various test settings.
    halSpiWriteReg(CCxxx0_IOCFG2,   0x29); // IOCFG2    GDO2 output pin configuration.
    halSpiWriteReg(CCxxx0_IOCFG0,   0x06); // IOCFG0D   GDO0 output pin configuration.    
    halSpiWriteReg(CCxxx0_PKTCTRL1, 0x04); // PKTCTRL1  Packet automation control.
    halSpiWriteReg(CCxxx0_PKTCTRL0, 0x05); // PKTCTRL0  Packet automation control.
    halSpiWriteReg(CCxxx0_ADDR,     0x00);   // ADDR      Device address.
    halSpiWriteReg(CCxxx0_PKTLEN,   0xFF); // PKTLEN    Packet length.

    #else // 1.2Kbps
    
    halSpiWriteReg(CCxxx0_FSCTRL1,  0x06);// FSCTRL1   Frequency synthesizer control.
    halSpiWriteReg(CCxxx0_FSCTRL0,  0x00);// FSCTRL0   Frequency synthesizer control.
    halSpiWriteReg(CCxxx0_FREQ2,    0x10);// FREQ2     Frequency control word, high byte.
    halSpiWriteReg(CCxxx0_FREQ1,    0xA7);// FREQ1     Frequency control word, middle byte.
    halSpiWriteReg(CCxxx0_FREQ0,    0x62);// FREQ0     Frequency control word, low byte.
    halSpiWriteReg(CCxxx0_MDMCFG4,  0xF5);// MDMCFG4   Modem configuration.
    halSpiWriteReg(CCxxx0_MDMCFG3,  0x83);// MDMCFG3   Modem configuration.
    halSpiWriteReg(CCxxx0_MDMCFG2,  0x02);// MDMCFG2   Modem configuration.
    halSpiWriteReg(CCxxx0_MDMCFG1,  0x22);// MDMCFG1   Modem configuration.
    halSpiWriteReg(CCxxx0_MDMCFG0,  0xF8);// MDMCFG0   Modem configuration.
    halSpiWriteReg(CCxxx0_CHANNR,   0x00);// CHANNR    Channel number.
    halSpiWriteReg(CCxxx0_DEVIATN,  0x15);// DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    halSpiWriteReg(CCxxx0_FREND1,   0x56);// FREND1    Front end RX configuration.
    halSpiWriteReg(CCxxx0_FREND0,   0x10);// FREND0    Front end RX configuration.
    halSpiWriteReg(CCxxx0_MCSM0 ,   0x18);// MCSM0     Main Radio Control State Machine configuration.
    halSpiWriteReg(CCxxx0_FOCCFG,   0x16);// FOCCFG    Frequency Offset Compensation Configuration.
    halSpiWriteReg(CCxxx0_BSCFG,    0x6C);// BSCFG     Bit synchronization Configuration.
    halSpiWriteReg(CCxxx0_AGCCTRL2, 0x03);// AGCCTRL2  AGC control.
    halSpiWriteReg(CCxxx0_AGCCTRL1, 0x40);// AGCCTRL1  AGC control.
    halSpiWriteReg(CCxxx0_AGCCTRL0, 0x91);// AGCCTRL0  AGC control.
    halSpiWriteReg(CCxxx0_FSCAL3,   0xE9);// FSCAL3    Frequency synthesizer calibration.
    halSpiWriteReg(CCxxx0_FSCAL2,   0x2A);// FSCAL2    Frequency synthesizer calibration.
    halSpiWriteReg(CCxxx0_FSCAL1,   0x00);// FSCAL1    Frequency synthesizer calibration.
    halSpiWriteReg(CCxxx0_FSCAL0,   0x1F);// FSCAL0    Frequency synthesizer calibration.
    halSpiWriteReg(CCxxx0_FSTEST,   0x59);// FSTEST    Frequency synthesizer calibration.
    halSpiWriteReg(CCxxx0_TEST2,    0x81);// TEST2     Various test settings.
    halSpiWriteReg(CCxxx0_TEST1,    0x35);// TEST1     Various test settings.
    halSpiWriteReg(CCxxx0_TEST0,    0x09);// TEST0     Various test settings.
    halSpiWriteReg(CCxxx0_IOCFG2,   0x29);// IOCFG2    GDO2 output pin configuration.
    halSpiWriteReg(CCxxx0_IOCFG0,   0x06);// IOCFG0D   GDO0 output pin configuration.    
    halSpiWriteReg(CCxxx0_PKTCTRL1, 0x04);// PKTCTRL1  Packet automation control.
    halSpiWriteReg(CCxxx0_PKTCTRL0, 0x05);// PKTCTRL0  Packet automation control.
    halSpiWriteReg(CCxxx0_ADDR,     0x00);// ADDR      Device address.
    halSpiWriteReg(CCxxx0_PKTLEN,   0xFF);// PKTLEN    Packet length.
    
    #endif
}
