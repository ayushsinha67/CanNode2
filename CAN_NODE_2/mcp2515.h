#ifndef _MCP2515_H
#define _MCP2515_H

#include <avr/io.h>
#include "can.h"

/************************************************************************
 *	MCP2515 Register Addresses
 */

/* Column 1 */
#define RXF0SIDH	0x00
#define RXF0SIDL	0x01
#define RXF0EID8	0x02
#define RXF0EID0	0x03
#define RXF1SIDH	0x04
#define RXF1SIDL	0x05
#define RXF1EID8	0x06
#define RXF1EID0	0x07
#define RXF2SIDH	0x08
#define RXF2SIDL	0x09
#define RXF2EID8	0x0A
#define RXF2EID0	0x0B
#define BFPCTRL		0x0C
#define TXRTSCTRL	0x0D
#define CANSTAT		0x0E
#define CANCTRL		0x0F

/* Column 2 */
#define RXF3SIDH	0x10
#define RXF3SIDL	0x11
#define RXF3EID8	0x12
#define RXF3EID0	0x13
#define RXF4SIDH	0x14
#define RXF4SIDL	0x15
#define RXF4EID8	0x16
#define RXF4EID0	0x17
#define RXF5SIDH	0x18
#define RXF5SIDL	0x19
#define RXF5EID8	0x1A
#define RXF5EID0	0x1B
#define TEC			0x1C
#define REC			0x1D

/* Column 3 */
#define RXM0SIDH	0x20
#define RXM0SIDL	0x21
#define RXM0EID8	0x22
#define RXM0EID0	0x23
#define RXM1SIDH	0x24
#define RXM1SIDL	0x25
#define RXM1EID8	0x26
#define RXM1EID0	0x27
#define CNF3		0x28
#define CNF2		0x29
#define CNF1		0x2A
#define CANINTE		0x2B
#define CANINTF		0x2C
#define EFLG		0x2D

/* Column 4 */
#define TXB0CTRL	0x30
#define TXB0SIDH	0x31
#define TXB0SIDL	0x32
#define TXB0EID8	0x33
#define TXB0EID0	0x34
#define TXB0DLC		0x35
#define TXB0D0		0x36
#define TXB0D1		0x37
#define TXB0D2		0x38
#define TXB0D3		0x39
#define TXB0D4		0x3A
#define TXB0D5		0x3B
#define TXB0D6		0x3C
#define TXB0D7		0x3D

/* Column 5 */
#define TXB1CTRL	0x40
#define TXB1SIDH	0x41
#define TXB1SIDL	0x42
#define TXB1EID8	0x43
#define TXB1EID0	0x44
#define TXB1DLC		0x45
#define TXB1D0		0x46
#define TXB1D1		0x47
#define TXB1D2		0x48
#define TXB1D3		0x49
#define TXB1D4		0x4A
#define TXB1D5		0x4B
#define TXB1D6		0x4C
#define TXB1D7		0x4D

/* Column 6 */
#define TXB2CTRL	0x50
#define TXB2SIDH	0x51
#define TXB2SIDL	0x52
#define TXB2EID8	0x53
#define TXB2EID0	0x54
#define TXB2DLC		0x55
#define TXB2D0		0x56
#define TXB2D1		0x57
#define TXB2D2		0x58
#define TXB2D3		0x59
#define TXB2D4		0x5A
#define TXB2D5		0x5B
#define TXB2D6		0x5C
#define TXB2D7		0x5D

/* Column 7 */
#define RXB0CTRL	0x60
#define RXB0SIDH	0x61
#define RXB0SIDL	0x62
#define RXB0EID8	0x63
#define RXB0EID0	0x64
#define RXB0DLC		0x65
#define RXB0D0		0x66
#define RXB0D1		0x67
#define RXB0D2		0x68
#define RXB0D3		0x69
#define RXB0D4		0x6A
#define RXB0D5		0x6B
#define RXB0D6		0x6C
#define RXB0D7		0x6D

/* Column 8 */
#define RXB1CTRL	0x70
#define RXB1SIDH	0x71
#define RXB1SIDL	0x72
#define RXB1EID8	0x73
#define RXB1EID0	0x74
#define RXB1DLC		0x75
#define RXB1D0		0x76
#define RXB1D1		0x77
#define RXB1D2		0x78
#define RXB1D3		0x79
#define RXB1D4		0x7A
#define RXB1D5		0x7B
#define RXB1D6		0x7C
#define RXB1D7		0x7D

/************************************************************************
 *	CONTROL REGISTER VALUES 
 */

/* CANCTRL - Control */ 
#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTENONLY 0x60
#define MODE_CONFIG     0x80
#define MODE_POWERUP	0xE0

#define ABORT_TX        0x10
#define MODE_ONESHOT	0x08

#define CLKOUT_ENABLE	0x04
#define CLKOUT_DISABLE	0x00
#define CLKOUT_PS1		0x00
#define CLKOUT_PS2		0x01
#define CLKOUT_PS4		0x02
#define CLKOUT_PS8		0x03

/* CNF1 - Bit Timing Configuration 1 */
#define SJW1            0x00
#define SJW2            0x40
#define SJW3            0x80
#define SJW4            0xC0

/* CNF2 - Bit Timing Configuration 2 */
#define BTLMODE			0x80
#define SAMPLE_1X       0x00
#define SAMPLE_3X       0x40

/* CNF3 - Bit Timing Configuration 3 */ 
#define SOF_ENABLE		0x80
#define SOF_DISABLE		0x00
#define WAKFIL_ENABLE	0x40
#define WAKFIL_DISABLE	0x00

/* CANINTF - Interrupt Flags */
#define RX0IF			0x01
#define RX1IF			0x02
#define TX0IF			0x04
#define TX1IF			0x08
#define TX2IF			0x10
#define ERRIF			0x20
#define WAKIF			0x40
#define MERRF			0x80

/* CANINTE - Interrupt Enable */
#define RX0IE			0x01
#define RX1IE			0x02
#define TX0IE			0x04
#define TX1IE			0x08
#define TX2IE			0x10
#define ERRIE			0x20
#define WAKIE			0x40
#define MERRE			0x80

/* EFLG - Error Flags */
#define EFLG_RX1OVR		0x80
#define EFLG_RX0OVR		0x40
#define EFLG_TXBO		0x20
#define EFLG_TXEP		0x10
#define EFLG_RXEP		0x08
#define EFLG_TXWAR		0x04
#define EFLG_RXWAR		0x02
#define EFLG_EWARN		0x01

/*TXBnCTRL - Transmit Control for Buffers 0,1,2
	� ABTF MLOA TXERR TXREQ � TXP1 TXP0
*/
#define TXB_TXBUFE    0x80
#define TXB_ABTF      0x40
#define TXB_MLOA      0x20
#define TXB_TXERR     0x10
#define TXB_TXREQ     0x08
#define TXB_TXP_11    0x03
#define TXB_TXP_10    0x02
#define TXB_TXP_01    0x01
#define TXB_TXP_00    0x00

/* TXRTSCTRL - RTS Pin Control and Status
	� � B2RTS B1RTS B0RTS B2RTSM B1RTSM B0RTSM
*/
#define TXB_RTS_SETALLINPUT	0x00
#define TXB0_RTS_SENDMSG		0x01
#define TXB1_RTS_SENDMSG		0x02
#define TXB2_RTS_SENDMSG		0x04
#define TXB0_RTS_DIGINPSTATE	0x08
#define TXB1_RTS_DIGINPSTATE	0x10
#define TXB2_RTS_DIGINPSTATE	0x20

/* TXBnSIDL - Extended Identifier Enable
	SID2 SID1 SID0 � EXIDE � EID17 EID16
*/
#define TXB_EXIDE			0x08

/* TXBnDLC - Remote Transmission Request Enable
	� RTR � � DLC3 DLC2 DLC1 DLC0
*/
#define TXB_RTR				0x40

/* RXB0CTRL - Receive Buffer 0 Control
	� RXM1 RXM0 � RXRTR BUKT BUKT1 FILHIT0
*/
#define RXB0_REC_ANY		0x60
#define RXB0_REC_EXIDE		0x40
#define RXB0_REC_SID		0x20
#define RXB0_REC_ALL_VALID	0x00
#define RXB0_RTR_RECEIVED	0x08
#define RXB0_ROLLOVER		0x04
#define RXB0_FILTERHIT0		0x00
#define RXB0_FILTERHIT1		0x01

/* RXB1CTRL
	� RXM1 RXM0 � RXRTR FILHIT2 FILHIT1 FILHIT0
*/
#define RXB1_REC_ANY		0x60
#define RXB1_REC_EXIDE		0x40
#define RXB1_REC_SID		0x20	
#define RXB1_REC_ALL_VALID	0x00
#define RXB1_RTR_RECIEVED	0x08
#define RXB1_FILTER5		0x05
#define RXB1_FILTER4		0x04
#define RXB1_FILTER3		0x03
#define RXB1_FILTER2		0x02
#define RXB1_FILTER1		0x01
#define RXB1_FILTER0		0x00

/* BFPCTRL � RXnBF PIN CONTROL AND STATUS
	� � B1BFS B0BFS B1BFE B0BFE B1BFM B0BFM
*/
#define RXB0_PIN_ENABLE		0x04
#define RXB1_PIN_ENABLE		0x08
#define RXB0_INTERRUPT		0x01
#define RXB1_INTERRUPT		0x02
#define RXB0_DIGINPSTATE	0x10
#define RXB1_DIGINPSTATE	0x20

/* RXBnSIDL � Standard Identifier Low 
SID2 SID1 SID0 SRR IDE � EID17 EID16
*/
#define RXB_REC_SRR			0x10
#define RXB_REC_EXIDE		0x08

/************************************************************************
 *	SPI COMMANDS
 */
#define RESET				0xC0	/* Read/Write */
#define WRITE				0x02
#define READ				0x03
#define BITMOD				0x05

#define READ_RXB0_SIDH		0x90	/* Read Receive Buffer */
#define READ_RXB0_D0		0x92
#define READ_RXB1_SIDH		0x94
#define READ_RXB1_D0		0x96

#define LOAD_TXB0_SIDH		0x40	/* Load Transmit Buffer */
#define LOAD_TXB0_D0		0x41
#define LOAD_TXB1_SIDH		0x42
#define LOAD_TXB1_D0		0x43
#define LOAD_TXB2_SIDH		0x44
#define LOAD_TXB2_D0		0x45

#define RTS_TXB0			0x81	/* Request to Send */
#define RTS_TXB1			0x82
#define RTS_TXB2			0x84
#define RTS_ALL				0x87

#define READ_STATUS			0xA0	/* Read Status */
#define RX_STATUS			0xB0


/************************************************************************
 *	INTERRUPTS
 */
#define ALL_TX_INT			0x1C	/* Enable all transmit interrupts */
#define TXB01_INT			0x0C	/* Enable TXB0 and TXB1 interrupts */
#define RX_INT				0x03	/* Enable receive interrupts */
#define NO_INT				0x00	/* Disable all interrupts */

/************************************************************************
 *	MASKS
 */
#define MODE_MASK			0xE0	/* Mode Mask */

#define RX0IF_STATUS		0x01	/* Read Status */
#define RX1IF_STATUS		0x02
#define TXB0_TXREQ_STATUS	0x04
#define TX0IF_STATUS		0x08
#define TXB1_TXREQ_STATUS	0x10
#define TX1IF_STATUS		0x20
#define TXB2_TXREQ_STATUS	0x40
#define TX2IF_STATUS		0x80

#define MSG_BUF_MASK		0xC0	/* Receive Status Masks */
#define MSG_TYPE_MASK		0x18
#define FILTER_MASK			0x07

/************************************************************************
 *	CAN SPEED
 *	PropSeg + PS1 >= PS2
 *	PropSeg + PS1 >= TDELAY
 *	PS2 > SJW
 * (PHSEG1 + 1) x TQ
 * (PRSEG + 1) x TQ
 * (PHSEG2 + 1) x TQ     [Minimum valid setting for PS2 is 2 TQ ]
 * TQ = (2 * (BRP + 1)) / F_CAN
 * SJW = 2 (normally), SJW = 1 ( 1 MBPS )
 */

#define F_OSC			16000		/* KBPS */
#define PROPSEG			0x01		/* 2TQ - 1 */
#define PHSEG1			0x30		/* 7TQ - 1 */
#define PHSEG2			0x05		/* 6TQ - 1 */
#define PROPSEG_1MBPS	0x01
#define PHSEG1_1MBPS	0x80
#define PHSEG2_1MBPS	0x01
#define CAN_20KBPS		20
#define CAN_25KBPS		25
#define CAN_50KBPS		50
#define CAN_100KBPS		100
#define CAN_125KBPS		125
#define CAN_250KBPS		250
#define CAN_500KBPS		500
#define CAN_750KBPS		750
#define CAN_1MBPS		1000

/************************************************************************
 *  DATA TYPES & ENUMERATIONS
 */
enum MCP2515_STATUS
{
	OK = 0,
	FAILED
};

/************************************************************************
 *	FUNCTION PROTOTYPES
 */

/* SPI Driver */
void	mcp2515_Reset			(void);
uint8_t mcp2515_Read			(const uint8_t address );
void	mcp2515_ReadRegs		(const uint8_t address, uint8_t data[], const uint8_t n );
void	mcp2515_Write			(const uint8_t address, const uint8_t data );
void	mcp2515_WriteRegs		(const uint8_t address, const uint8_t data[], const uint8_t n );
void    mcp2515_BitModify		(const uint8_t address, const uint8_t mask, const uint8_t data );
void	mcp2515_StrRXBUF		(const uint8_t buffer, uint8_t data[], const uint8_t n );
void	mcp2515_LoadTXBUF		(const uint8_t buffer, const uint8_t data[], const uint8_t n );
void    mcp2515_RTS				(const uint8_t buffer );
uint8_t	mcp2515_ReadStatus		(void);
uint8_t	mcp2515_RXStatus		(void);

/* Configurations */
enum MCP2515_STATUS	mcp2515_Init		(const uint8_t can_rate );
enum MCP2515_STATUS	mcp2515_SetMode		(const uint8_t mode );
enum MCP2515_STATUS	mcp2515_ConfigRate	(const uint16_t can_rate );
void				mcp2515_ClrBuffers	(void);
void				mcp2515_ConfigPins  (void);
void				mcp2515_ConfigFilt  (void);

/* Buffer Message Access */
void				mcp2515_ReadRxBuf		(CanMessage *m, uint8_t addr );
void				mcp2515_WriteTxBuf		(const CanMessage *m, uint8_t addr );
enum MCP2515_STATUS mcp2515_ChkFreeTxBuf	(uint8_t addr[] );

#endif