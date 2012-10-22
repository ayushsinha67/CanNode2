#include "message.h"
#include "messagedef.h"
#include "mcp2515.h"
#include "can.h"
#include "init.h"
#include "adc.h"
#include "buffer.h"

#if ( TERMINAL == 1 )
#include <avr/pgmspace.h>
#include "uart.h"
#include "terminal.h"
#endif

/************************************************************************
 *	GLOBAL VARIABLES
 */
/* CAN Messages */
extern CanMessage PneumaticShift, Oil, Brake;

/* Buffer */
extern volatile CanBuffer  RxBuffer;
extern volatile CanBuffer  TxBuffer;

/************************************************************************
 *	INITIALIZE MESSAGE TO BE TRANSMITTED
 */
void Msg_Init( void )
{
	/* Pneumatic Gear Shift */
	PneumaticShift.ext = CAN_MODE;
	PneumaticShift.id  = PNEUM_ID;
	PneumaticShift.dlc = PNEUM_DLC;
	PneumaticShift.data[0] = 127;
	
	/* OIL */
	Oil.ext = CAN_MODE;
	Oil.id = OIL_ID;
	Oil.dlc = OIL_DLC;
	Oil.data[0] = 0x23;
	Oil.data[1] = 0xAB;
	Oil.data[2] = 0xCD;
	Oil.data[3] = 0x81;
	
	/* BRAKE */
	Brake.ext = CAN_MODE;
	Brake.id = BRAKE_ID;
	Brake.dlc = BRAKE_DLC;
	Brake.data[0] = 0xCA;
	Brake.data[1] = 0xDE;
}

/************************************************************************
 *	CHECK MESSAGES RECEIVED
 */
void Msg_Chk( CanMessage *msg )
{
	switch( msg->id ){
		
		case PNEUM_ID: Pneumatic_RecMsg( msg ); break;
			  default: break;
	}
}

/************************************************************************
 *	SEND PNEUMATIC MESSAGE
 */
void Pneumatic_SendMsg( uint8_t msg )
{
	if( CAN_BufState( &TxBuffer ) != BUFFER_FULL ){
		PneumaticShift.data[0] = msg;
		CAN_BufEnq( &TxBuffer, &PneumaticShift );	
	}	
}

/************************************************************************
 *	RECEIVE PNEUMATIC MESSAGE
 */
void Pneumatic_RecMsg( CanMessage *m )
{
	/* Pneumatic */
	if( m->data[0] == PNEUM_MSG_UPSHIFT )
		UART_TxStr_p( PSTR("UP\n") );
		
	else if ( m->data[0] == PNEUM_MSG_DOWNSHIFT )
		UART_TxStr_p( PSTR("DOWN\n") );
}

