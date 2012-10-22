/*
 * CAN_NODE_2.c
 * 
 * Status: NODE 2
 * 
 * Created: 01-06-2012 AM 11:11:52
 *  Author: Ayush Sinha
 */ 
#define  F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <avr/wdt.h>
#include "init.h"
#include "spi.h"
#include "adc.h"
#include "uart.h"
#include "mcp2515reg.h"
#include "mcp2515.h"
#include "can.h"
#include "buffer.h"
#include "messagedef.h"
#include "message.h"
#include "pneumatic.h"

#if ( TERMINAL == 1 )
#include "uart.h"
#include "terminal.h"							
extern volatile MSGSTRM_STATE	strm;
#endif

/************************************************************************
 *	GLOBALS
 */
/* Messages */
volatile CanMessage rec;
CanMessage PneumaticShift, Oil, Brake;

/* Buffer */
volatile CanBuffer  RxBuffer;
volatile CanBuffer  TxBuffer;

/* Timer */
volatile uint16_t	tx_timer = 0;
volatile uint16_t	tick_msg1 = 0, tick_msg2 = 0,
					tick_msg3 = 0, tick_msg4 = 0, tick_msg5 = 0, 
					tick_msg6 = 0, tick_msg7 = 0;
					
/* Pneumatic */
volatile uint16_t debounce_ticker1 = 0;
volatile uint16_t debounce_ticker2 = 0;
volatile ButtonState button_int1 = BUTTON_RELEASED;
volatile ButtonState button_int2 = BUTTON_RELEASED;

/************************************************************************
 *	MAIN
 */

int main(void)
{	 
	CanMessage tmp;											/* Local Variables */

#if ( TERMINAL == 1 )
	UART_Init();											/* UART */
	ADC_Init();												/* ADC */
#endif	
												
	GPIO_Init();											/* GPIO */
	ExtINT_Init();											/* External Interrupt */
	Timer_Init();											/* Timers */											
	SPI_Init();												/* SPI */
											
	CanStatus res = CAN_Init(CAN_SPEED);					/* Start CAN */	
	
#if ( TERMINAL == 1 )		
	term_Start(res);										/* Start Terminal */
#endif
	
	Msg_Init();												/* Construct Messages to be sent */
	
	CAN_BufInit( &RxBuffer, CAN_RX_BUFFER_SIZE );			/* Initialize Receive Buffer */
	CAN_BufInit( &TxBuffer, CAN_TX_BUFFER_SIZE );			/* Initialize Transmit Buffer */
	
	sei();													/* Enable Global Interrupts */

/* ---------------------------*/
	while(1){
		
		wdt_enable(WDTO_1S);								/* Enable Watchdog Timer for 2 second */
		
		/* ------------------------------------------ */
		/* Send Messages */
		
		// Get Data
		
		ATOMIC_BLOCK( ATOMIC_FORCEON ){
			
			if( CAN_BufState( &TxBuffer ) != BUFFER_EMPTY ){/* Check if empty */
				
				CAN_BufDeq( &TxBuffer, &tmp );				/* Dequeue */
				CAN_SendMsg( &tmp );						/* Send */
			}
			
		}										
		
		/* ------------------------------------------ */		
		/* Receive Messages */
		
		ATOMIC_BLOCK( ATOMIC_FORCEON ){						/* Read Interrupt variables */
			
			if( CAN_BufState( &RxBuffer ) != BUFFER_EMPTY ){/* Check if not empty */
		
				CAN_BufDeq( &RxBuffer, &tmp );				/* Dequeue */
				
				Msg_Chk( &tmp );							/* Check Received Message */		

#if ( TERMINAL == 1 )		
				if( strm == MS_STREAM )						/* Enable Terminal Message Stream */
					term_RxMsg( &tmp );
#endif
			}										
		}				

		/* ------------------------------------------ */
		
#if ( TERMINAL == 1 )											
	term_Main();											/* TERMINAL FOR DEBUGGING */
#endif

    }
	
	wdt_reset();											/* Reset Watchdog */
	wdt_disable();
	
	return 0;
}

/************************************************************************
 *	INT0 - CAN RECEIVED MESSAGE 
 */
ISR(INT0_vect)
{	
	CAN_ReadMsg( &rec );									/* Read Message Received */
	
	if( CAN_BufState( &RxBuffer ) != BUFFER_FULL )		
		CAN_BufEnq( &RxBuffer, &rec );						/* Enqueue to Receive Buffer */
	
}
 
/************************************************************************
 *	TIMER0 INTERRUPT (1 MS)
 */ 
 ISR(TIMER0_COMP_vect)
{
	if( tx_timer > 0 )										/* Transmit decrement timer  */
		tx_timer--;
	
	/* Enqueue into Transmit Buffer */	
	if( tick_msg1++ >= RATE_MSG1 ){							/* Message 1 */
		
		if( CAN_BufState( &TxBuffer ) != BUFFER_FULL )
			CAN_BufEnq( &TxBuffer, &Oil );
		
		tick_msg1 = 0;	
	}	
	
	if( tick_msg2++ >= RATE_MSG2 ){							/* Message 2 */
		
		if( CAN_BufState( &TxBuffer ) != BUFFER_FULL )
			CAN_BufEnq( &TxBuffer, &Brake );	
		
		tick_msg2 = 0;		
	}
	
	if( tick_msg3++ >= RATE_MSG3 ){							/* Message 3 */
		
		if( CAN_BufState( &TxBuffer ) != BUFFER_FULL )
			//CAN_BufEnq( &TxBuffer, &throttle );	
		
		tick_msg3 = 0;		
	}
	
	if( tick_msg4++ >= RATE_MSG4 ){							/* Message 4 */
		
		if( CAN_BufState( &TxBuffer ) != BUFFER_FULL )
			//CAN_BufEnq( &TxBuffer, &throttle );	
		
		tick_msg4 = 0;		
	}
	
	if( tick_msg5++ >= RATE_MSG5 ){							/* Message 5 */
		
		if( CAN_BufState( &TxBuffer ) != BUFFER_FULL )
			//CAN_BufEnq( &TxBuffer, &throttle );	
		
		tick_msg5 = 0;		
	}
	
	if( tick_msg6++ >= RATE_MSG6 ){							/* Message 6 */
		
		if( CAN_BufState( &TxBuffer ) != BUFFER_FULL )
			//CAN_BufEnq( &TxBuffer, &throttle );	
		
		tick_msg6 = 0;		
	}
	
	if( tick_msg7++ >= RATE_MSG7 ){							/* Message 7 */
		
		if( CAN_BufState( &TxBuffer ) != BUFFER_FULL )
			//CAN_BufEnq( &TxBuffer, &throttle );	
		
		tick_msg7 = 0;		
	}
}

/************************************************************************
 *	INT1 INTERRUPT - PNEUMATIC BUTTON 1
 */
ISR( INT1_vect )
{	
	/* Noise detected, since next edge is less than debounce time. */
	if( ( button_int1 == BUTTON_CHECK_PRESSED ) && ( debounce_ticker1 < DEBOUNCE_TIME ) ){		
		button_int1 = BUTTON_RELEASED;	
	}
	
	/* Noise detected, since next edge is less than debounce time */
	if( ( button_int1 == BUTTON_CHECK_RELEASED ) && ( debounce_ticker1 < DEBOUNCE_TIME ) ){	
		button_int1 = BUTTON_PRESSED;
	}	
	
	/* Start incrementing timer if key was released and is now pressed. This can be noise or valid */
	if( button_int1 == BUTTON_RELEASED ){					
		debounce_ticker1 = 0;
		button_int1 = BUTTON_CHECK_PRESSED;		
	}	
	
	/* Start incrementing timer if key was pressed. This is the trailing noise edge. */
	if( button_int1 == BUTTON_PRESSED ){
		debounce_ticker1 = 0;
		button_int1 = BUTTON_CHECK_RELEASED;	
	}
}

/************************************************************************
 *	INT2 INTERRUPT - PNEUMATIC BUTTON 2
 */
ISR( INT2_vect )
{	
	/* Noise detected, since next edge is less than debounce time. */
	if( ( button_int2 == BUTTON_CHECK_PRESSED ) && ( debounce_ticker2 < DEBOUNCE_TIME ) ){	
		button_int2 = BUTTON_RELEASED;	
	}
	
	/* Noise detected, since next edge is less than debounce time */
	if( ( button_int2 == BUTTON_CHECK_RELEASED ) && ( debounce_ticker2 < DEBOUNCE_TIME ) ){	
		button_int2 = BUTTON_PRESSED;
	}	
	
	/* Start incrementing timer if key was released and is now pressed. This can be noise or valid */
	if( button_int2 == BUTTON_RELEASED ){					
		debounce_ticker2 = 0;
		button_int2 = BUTTON_CHECK_PRESSED;		
	}	
	
	/* Start incrementing timer if key was pressed. This is the trailing noise edge. */
	if( button_int2 == BUTTON_PRESSED ){
		debounce_ticker2 = 0;
		button_int2 = BUTTON_CHECK_RELEASED;	
	}
}

/************************************************************************
 *	TIMER2 INTERRUPT (1 MS)
 */ 
 ISR(TIMER2_COMP_vect)
 {
	/* DEBOUNCE INT1 -------------------------------------------- */
	
	/* Increment ticker if signal is high and in "pressed" check state */	
	if( ( button_int1 == BUTTON_CHECK_PRESSED ) && ( CHK( PIN_SHIFT_BUTTON_UP, 1<<PIN_SHIFT_UP ) ) ){
		debounce_ticker1++;
	}
	
	/* Increment ticker if signal is low and in "released" check state */
	if( ( button_int1 == BUTTON_CHECK_RELEASED ) && ( !CHK( PIN_SHIFT_BUTTON_UP, 1<<PIN_SHIFT_UP ) ) ){
		debounce_ticker1++;	
	}
	
	/* If there was no trailing noise edge on button release */
	if( ( button_int1 == BUTTON_PRESSED ) && ( !CHK( PIN_SHIFT_BUTTON_UP, 1<<PIN_SHIFT_UP ) ) ){
		
		button_int1 = BUTTON_CHECK_RELEASED;
		debounce_ticker1 = 0;
	}
	
	/* A valid button press is detected */	
	if( ( button_int1 == BUTTON_CHECK_PRESSED ) && ( debounce_ticker1 >= DEBOUNCE_TIME ) ){
		
		button_int1 = BUTTON_PRESSED;
		debounce_ticker1 = 0;
		
		/* BUTTON IS PRESSED */
		UART_TxStr_p( PSTR("UP SHIFT\n") );
		Pneumatic_SendMsg ( PNEUM_MSG_UPSHIFT );					/* Send Message */
	}
	
	/* A valid button release is detected */
	if( ( button_int1 == BUTTON_CHECK_RELEASED ) && ( debounce_ticker1 >= DEBOUNCE_TIME ) ){
		button_int1 = BUTTON_RELEASED;
	} 	 
	 	
	/* DEBOUNCE INT2 ----------------------------------------- */
		
	/* Increment ticker if signal is high and in "pressed" check state */
	if( ( button_int2 == BUTTON_CHECK_PRESSED ) && ( CHK( PIN_SHIFT_BUTTON_DOWN, 1<<PIN_SHIFT_DOWN ) ) ){
		debounce_ticker2++;
	}
	
	/* Increment ticker if signal is low and in "released" check state */
	if( ( button_int2 == BUTTON_CHECK_RELEASED ) && ( !CHK( PIN_SHIFT_BUTTON_DOWN, 1<<PIN_SHIFT_DOWN ) ) ){
		debounce_ticker2++;	
	}
	
	/* If there was no trailing noise edge on button release */
	if( ( button_int2 == BUTTON_PRESSED ) && ( !CHK( PIN_SHIFT_BUTTON_DOWN, 1<<PIN_SHIFT_DOWN ) ) ){
		button_int2 = BUTTON_CHECK_RELEASED;
		debounce_ticker2 = 0;
	}
	
	/* A valid button press is detected */		
	if( ( button_int2 == BUTTON_CHECK_PRESSED ) && ( debounce_ticker2 >= DEBOUNCE_TIME ) ){
		
		button_int2 = BUTTON_PRESSED;
		debounce_ticker2 = 0;
		
		/* BUTTON IS PRESSED */				
		UART_TxStr_p( PSTR("DOWN SHIFT\n") );
		Pneumatic_SendMsg( PNEUM_MSG_DOWNSHIFT );					/* Send Message */	
	}
	
	/* A valid button release is detected */
	if( ( button_int2 == BUTTON_CHECK_RELEASED ) && ( debounce_ticker2 >= DEBOUNCE_TIME ) ){
		button_int2 = BUTTON_RELEASED;
	} 	 
 }
 