/*************************************************************************
Author:   	$Author: dennis $
File:     	$HeadURL: file://localhost/home/dennis/svn-store/avr-source/gpib_004/timer16.c $
Date:  		$Date: 2008-05-11 07:57:02 +0200 (So, 11 Mai 2008) $ 
Revision: 	$Revision: 65 $ 
Id: 		$Id: timer16.c 65 2008-05-11 05:57:02Z dennis $ 
Licence:	GNU General Public License
 *************************************************************************/

/** 
 *  @defgroup timer_lib Timer Library
 *  @code #include <timer16.h> @endcode
 * 
 *  @brief Timer Library. 
 *
 *  16 bit timer implementation. 
 *
 *  @author Dennis Dingeldein  http://www.dingeldein-online.de
 */

#include "timer16.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart.h"

volatile uint8_t timer; /**< timer value, increased by interrupt */
volatile uint8_t s; // second value, increased by one every second

/**
 * Interrupt Service Routine - do not call directly
 *  This routine is called when the Timer Value TCNT1 reaches the Output Compare Register Value OCR1A
 */ 
ISR(TIMER1_COMPA_vect) {
#if XTAL_CPU % DEBOUNCE
	OCR1A = XTAL_CPU / DEBOUNCE - 1;
#endif
	if (--timer==0) {
		timer=(uint8_t) DEBOUNCE;
		s++;
#if XTAL_CPU % DEBOUNCE
		OCR1A = XTAL_CPU / DEBOUNCE + XTAL_CPU % DEBOUNCE - 1;
#endif
	}
}

/**
 * Timer initialisation.
 *  \brief initializes all registers ands sets second value 's' to zero. Timer is immediately
 * started.
 */ 
void timer16_init( void ) {
	// Initialisierung:
	// (1<<CS10) : Timer1 Vorteiler = 001 = 1. Der Zähler wird also mit f=8Mhz hochgezählt

	// OCR1A=XTAL/DEBOUNCE-1 -> Bei Erreichen dieses Wertes (31249) wird die ISR besucht
	// die ISR wird also alle 1/256s aufgerufen. Wenn man dort also bis 256 hochzählt, ist genau
	// 1 Sekunde um! 256 ist der Wert der Variable timer und wird über das define DEBOUNCE festgelegt

	TCCR1B = (1<<CS10) ^ (1<<WGM12);	// Prescaler of 1 | CTC mode
	OCR1A  = F_CPU/DEBOUNCE-1;		// Output compare register value 
	TCNT1 = 0; // Start value for timer register
	s=0; // Initialize second value (s) to zero
	timer = (uint8_t)DEBOUNCE; 
	TIMSK |= (1<<OCIE1A);		// activate timer interrupts which starts timer run
}
