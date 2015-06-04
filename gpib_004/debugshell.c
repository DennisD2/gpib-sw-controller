/*************************************************************************
 Author:   	$Author: dennis $
 File:     	$HeadURL: file://localhost/home/dennis/svn-store/avr-source/gpib_004/debugshell.c $
 Date:  		$Date: 2012-04-16 21:14:27 +0200 (Mo, 16 Apr 2012) $ 
 Revision: 	$Revision: 694 $ 
 Id: 		$Id: debugshell.c 694 2012-04-16 19:14:27Z dennis $ 
 Licence:	GNU General Public License

 DESCRIPTION:
 debug shell implementation. 
 *************************************************************************/

#ifdef USE_DEBUGSHELL
/** 
 *  @defgroup gpib_lib GPIB Library
 *  @code #include <gpib.h> @endcode
 * 
 *  @brief GPIB library. 
 *
 *  This library can be used to transmit and receive data through GPIB bus. 
 *
 *
 *  @author Dennis Dingeldein  http://www.dingeldein-online.de
 */

#include "string.h"

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "gpib.h"
#include "uart.h"
#include "debugshell.h"

/**
// Debug shell function
 * \brief this function is called if timeout situations occur in the code.
 * The debugshell allows to manually check the state of the lines. If they are allowed
 * to be changed manually, this should be implemented here too.
*/
void debugshell( void ) {
	unsigned int c;
	uchar ch;
	uchar buf[32];
	int buf_ptr=0;
	uchar end_loop=0;

	/*
	 * Get received character from ringbuffer
	 * uart_getc() returns in the lower byte the received character and 
	 * in the higher byte (bitmask) the last receive error
	 * UART_NO_DATA is returned when no data is available.
	 *
	 */

	uart_puts("DEBUGSHELL commands\n\r");
	uart_puts("i - dump info in GPIB data lines\n\r");
	uart_puts("x - exit debug shell\n\r");
	uart_puts("DEBUG> ");
	while (!end_loop) {
		c = uart_getc();
		if ( c & UART_NO_DATA ) {
			// no data available from UART
			continue;
		}

		/*
		 * new data available from UART
		 * check for Frame or Overrun error
		 */
		if ( c & UART_FRAME_ERROR ) {
			/* Framing Error detected, i.e no stop bit detected */
			uart_puts_P("UART Frame Error: ");
		}
		if ( c & UART_OVERRUN_ERROR ) {
			/* 
			 * Overrun, a character already present in the UART UDR register was 
			 * not read by the interrupt handler before the next character arrived,
			 * one or more received characters have been dropped
			 */
			uart_puts_P("UART Overrun Error: ");
		}
		if ( c & UART_BUFFER_OVERFLOW ) {
			/* 
			 * We are not reading the receive buffer fast enough,
			 * one or more received character have been dropped 
			 */
			uart_puts_P("Buffer overflow error: ");
		}

		/* 
		 * send received character back
		 */
		uart_putc( (unsigned char)c );

		// make uchar from character in int value
		ch = (uchar) c;
		// add to buffer
		buf[buf_ptr++] = ch;
		// terminate string
		buf[buf_ptr] = '\0';

		// <CR> means command input is complete
		if (ch==0x0d) {
			// adjust string terminator
			buf[--buf_ptr]='\0';

			if (strcmp((char*)buf,"x")==0) {
				end_loop=1;
			}
			if (strcmp((char*)buf,"i")==0) {
				uart_puts("\n\r");
				gpib_info();
			}
			// here: additional commands

			buf_ptr=0;
			uart_puts("DEBUG> ");
		}
	}
	uart_puts("\r\n");
}
#endif
