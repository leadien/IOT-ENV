#ifndef __CC1100_H__
#define __CC1100_H__
//#include "spca_general.h"
#include "def.h"
//------------------------------------------------------------------------------------------------------
// CC2500/CC1100 STROBE, CONTROL AND STATUS REGSITER
#define CCxxx0_IOCFG2       0x00        // GDO2 output pin configuration
#define CCxxx0_IOCFG1       0x01        // GDO1 output pin configuration
#define CCxxx0_IOCFG0       0x02        // GDO0 output pin configuration
#define CCxxx0_FIFOTHR     0x03       // RX FIFO and TX FIFO thresholds
#define CCxxx0_SYNC1         0x04       // Sync word, high byte
#define CCxxx0_SYNC0         0x05       // Sync word, low byte
#define CCxxx0_PKTLEN       0x06       // Packet length
#define CCxxx0_PKTCTRL1   0x07       // Packet automation control
#define CCxxx0_PKTCTRL0   0x08       // Packet automation control
#define CCxxx0_ADDR          0x09       // Device address
#define CCxxx0_CHANNR      0x0A       // Channel number
#define CCxxx0_FSCTRL1     0x0B       // Frequency synthesizer control
#define CCxxx0_FSCTRL0     0x0C       // Frequency synthesizer control
#define CCxxx0_FREQ2         0x0D      // Frequency control word, high byte
#define CCxxx0_FREQ1         0x0E      // Frequency control word, middle byte
#define CCxxx0_FREQ0         0x0F      // Frequency control word, low byte
#define CCxxx0_MDMCFG4    0x10      // Modem configuration
#define CCxxx0_MDMCFG3    0x11      // Modem configuration
#define CCxxx0_MDMCFG2    0x12      // Modem configuration
#define CCxxx0_MDMCFG1    0x13      // Modem configuration
#define CCxxx0_MDMCFG0    0x14      // Modem configuration
#define CCxxx0_DEVIATN     0x15      // Modem deviation setting
#define CCxxx0_MCSM2        0x16      // Main Radio Control State Machine configuration
#define CCxxx0_MCSM1        0x17      // Main Radio Control State Machine configuration
#define CCxxx0_MCSM0        0x18      // Main Radio Control State Machine configuration
#define CCxxx0_FOCCFG       0x19      // Frequency Offset Compensation configuration
#define CCxxx0_BSCFG         0x1A      // Bit Synchronization configuration
#define CCxxx0_AGCCTRL2   0x1B      // AGC control
#define CCxxx0_AGCCTRL1   0x1C      // AGC control
#define CCxxx0_AGCCTRL0   0x1D      // AGC control
#define CCxxx0_WOREVT1    0x1E      // High byte Event 0 timeout
#define CCxxx0_WOREVT0    0x1F      // Low byte Event 0 timeout
#define CCxxx0_WORCTRL    0x20      // Wake On Radio control
#define CCxxx0_FREND1       0x21      // Front end RX configuration
#define CCxxx0_FREND0       0x22      // Front end TX configuration
#define CCxxx0_FSCAL3       0x23      // Frequency synthesizer calibration
#define CCxxx0_FSCAL2       0x24      // Frequency synthesizer calibration
#define CCxxx0_FSCAL1       0x25      // Frequency synthesizer calibration
#define CCxxx0_FSCAL0       0x26      // Frequency synthesizer calibration
#define CCxxx0_RCCTRL1     0x27      // RC oscillator configuration
#define CCxxx0_RCCTRL0     0x28      // RC oscillator configuration
#define CCxxx0_FSTEST       0x29      // Frequency synthesizer calibration control
#define CCxxx0_PTEST         0x2A      // Production test
#define CCxxx0_AGCTEST    0x2B      // AGC test
#define CCxxx0_TEST2         0x2C      // Various test settings
#define CCxxx0_TEST1         0x2D      // Various test settings
#define CCxxx0_TEST0         0x2E      // Various test settings

// Strobe commands
#define CCxxx0_SRES           0x30        // Reset chip.
#define CCxxx0_SFSTXON    0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                                           // If in RX/TX: Go to a wait state where only the synthesizer is
                                                           // running (for quick RX / TX turnaround).
#define CCxxx0_SXOFF        0x32        // Turn off crystal oscillator.
#define CCxxx0_SCAL          0x33        // Calibrate frequency synthesizer and turn it off
                                                           // (enables quick start).
#define CCxxx0_SRX            0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                                           // MCSM0.FS_AUTOCAL=1.
#define CCxxx0_STX            0x35        // In IDLE state: Enable TX. Perform calibration first if
                                                           // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                                           // Only go to TX if channel is clear.
#define CCxxx0_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                                          // Wake-On-Radio mode if applicable.
#define CCxxx0_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CCxxx0_SWOR        0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CCxxx0_SPWD        0x39        // Enter power down mode when CSn goes high.
#define CCxxx0_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CCxxx0_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CCxxx0_SWORRST  0x3C        // Reset real time clock.
#define CCxxx0_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                                          // bytes for simpler software.

#define CCxxx0_PARTNUM       0x30
#define CCxxx0_VERSION         0x31
#define CCxxx0_FREQEST        0x32
#define CCxxx0_LQI                 0x33
#define CCxxx0_RSSI               0x34
#define CCxxx0_MARCSTATE    0x35
#define CCxxx0_WORTIME1     0x36
#define CCxxx0_WORTIME0     0x37
#define CCxxx0_PKTSTATUS    0x38
#define CCxxx0_VCO_VC_DAC  0x39
#define CCxxx0_TXBYTES         0x3A
#define CCxxx0_RXBYTES         0x3B

#define CCxxx0_PATABLE         0x3E
#define CCxxx0_TXFIFO           0x3F
#define CCxxx0_RXFIFO            0x3F

typedef unsigned char BYTE;


//void SPI_Init(void);
void SPI_CC1100_Init(void);
void CC1100Init(void);
BYTE SPI_RxTxByte(BYTE data);
BYTE halSpiReadStatus(BYTE addr);
BYTE halSpiReadReg(BYTE addr);
void halSpiWriteReg(BYTE addr, BYTE value);
void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count);	
void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count);
void halSpiStrobe(BYTE strobe);
void RESET_CCxxx0(void);
void halRfWriteRfSettings(void);

// Defines
#define WRITE_BURST     0x40
#define READ_SINGLE     0x80
#define READ_BURST      0xC0

//#define ADDR        0

// Defines used by the displayMenu() function to keep track of the current menu entry
#define PACKET_LENGTH       0
#define NUMBER_OF_PACKETS   1
#define RADIO_MODE          2
#define START               3

// defines used for assigning values to the variables in the MENU_DATA struct.
#define TX  0
#define RX  1

// defines used in the state machine in the main loop
#define TX_START    0
#define RX_START    1
#define SETUP       2

#define FIFO_SIZE   64

#define CRC_OK  0x80
//#define SYNC            0
//#define END_OF_PACKET   1
#define SYNC            0
#define END_OF_PACKET   1 

#define AVAILABLE_BYTES_IN_TX_FIFO  60
#define BYTES_IN_RX_FIFO            60

#define PACKET_INT      GDO0 

// Struct. used to hold information used for packet handling in TX
typedef struct TX_DATA {
    UINT8 packetLength;              // packet length
    UINT16 bytesLeft;           // Used to keep track of how many bytes are left to be written to 
                                // the TX FIFO
    UINT8 iterations;           // For packets greater than 64 bytes, this variable is used to keep 
                                // track of how many time the TX FIFO should be re-filled to its limit 
    BOOL writeRemainingDataFlag;// When this flag is set, the TX FIFO should not be filled entirely
    BOOL packetSentFlag;        // Flag set when GDO0 indicates that the packet is sent
    UINT8 *pBufferIndex;        // Pointer to current position in the txBuffer 
    UINT16 packetsSent;         // Number of packets transmitted
} TX_DATA; 

// Struct. used to hold information used for packet handling in RX
typedef struct RX_DATA {
    UINT16 bytesLeft;           // Used to keep track of how many bytes are left to be read from the RX FIFO
    BOOL packetReceivedFlag;    // Flag set when GDO0 indicates that a packet is received
    BOOL syncOrEndOfPacket;     // Flag used to determine if the interrupt is caused by a rising or
                                // a falling edge
    UINT8 *pBufferIndex;        // Pointer to current position in the rxBuffer 
    UINT8 lengthByte;           // LengthByte (This example require variable packet length mode)
    BOOL crcOK;                 // CRC_OK flag from status byte in RX FIFO
    UINT16 packetsReceived;     // Number of packets received
} RX_DATA;


#define SPI_CC1100                 SPI1
#define SPI_CC1100_CLK             RCC_APB2Periph_SPI1
#define SPI_CC1100_GPIO            GPIOA
#define SPI_CC1100_GPIO_CLK        RCC_APB2Periph_GPIOA
#define SPI_CC1100_PIN_SCK         GPIO_Pin_5
#define SPI_CC1100_PIN_MISO        GPIO_Pin_6
#define SPI_CC1100_PIN_MOSI        GPIO_Pin_7

#define SPI_CC1100_CS             GPIO_Pin_0
#define SPI_CC1100_CS_GPIO        GPIOB
#define SPI_CC1100_CS_GPIO_CLK    RCC_APB2Periph_GPIOB
#if 0
/* Exported macro ------------------------------------------------------------*/
/* Select SPI CC1100: Chip Select pin low  */
#define SPI_CC1100_CS_LOW()       GPIO_ResetBits(SPI_CC1100_CS_GPIO, SPI_CC1100_CS)
/* Deselect SPI CC1100: Chip Select pin high */
#define SPI_CC1100_CS_HIGH()      GPIO_SetBits(SPI_CC1100_CS_GPIO, SPI_CC1100_CS)
#define SPI_CC1100_CS_STAT()	GPIO_ReadOutputDataBit(SPI_CC1100_CS_GPIO,SPI_CC1100_CS)
#else
#define SPI_CC1100_CS_LOW()		(SPI_CC1100_CS_GPIO->BRR = SPI_CC1100_CS)
/* Deselect SPI CC1100: Chip Select pin high */
#define SPI_CC1100_CS_HIGH()	(SPI_CC1100_CS_GPIO->BSRR = SPI_CC1100_CS)
#define SPI_CC1100_CS_STAT()	(SPI_CC1100_CS_GPIO->ODR & SPI_CC1100_CS)
#endif
//GPIO_TypeDef tmp;
#define SPI_CC1100_SO_IS_HIGH()	(SPI_CC1100_GPIO->IDR&SPI_CC1100_PIN_MISO)

#define MRFI_GDO0_PIN	GPIO_Pin_0
#define MRFI_GDO2_PIN	GPIO_Pin_4
#define MRFI_GDO0_LINE		EXTI_Line0
#define MRFI_GDO2_LINE		EXTI_Line4
//#define MRFI_GDO0_PIN_LINE		MRFI_GDO0_PIN
//#define MRFI_GDO2_PIN_LINE		MRFI_GDO2_PIN
#define MRFI_GDO0_PIN_SOURCE		GPIO_PinSource0
#define MRFI_GDO2_PIN_SOURCE		GPIO_PinSource4
#define MRFI_GDO0_IRQ_CHANNEL	EXTI0_IRQn
#define MRFI_GDO1_IRQ_CHANNEL	EXTI4_IRQn
#define MRFI_GDO_PORT	GPIOA
#define MRFI_GDO_PORT_CLK		RCC_APB2Periph_GPIOA
#define MRFI_GDO_PORT_SOURCE	GPIO_PortSourceGPIOA
#define MRFI_TRIGGER		EXTI_Trigger_Falling

#define MRFI_TX_EN_PIN		GPIO_Pin_14
#define MRFI_RX_EN_PIN		GPIO_Pin_15
#define MRFI_TR_EN_PORT		GPIOG
#define MRFI_TR_EN_CLK			RCC_APB2Periph_GPIOG

#endif
