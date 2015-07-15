/*************************************************************************
 Author:   	$Author: dennis $
 File:     	$HeadURL: file://localhost/home/dennis/svn-store/avr-source/gpib_004/main_gpib.c $
 Date:  		$Date: 2012-04-19 16:33:45 +0200 (Do, 19 Apr 2012) $
 Revision: 	$Revision: 696 $
 Id: 		$Id: main_gpib.c 696 2012-04-19 14:33:45Z dennis $
 Licence:	GNU General Public License
 *************************************************************************/
/** 
 *  @defgroup gpib_lib GPIB Library
 *  @code #include <gpib.h> @endcode
 * 
 *  @brief GPIB library. 
 *
 *  This library can be used to transmit and receive data through GPIB bus. 
 *
 *  @author Dennis Dingeldein  http://www.dingeldein-online.de
 */

/*! \mainpage GPIB controller implementation with AVR
 *
 * \section intro_sec Introduction
 * These files implement a GPIB controller. No special hardware is used, the port
 * lines of the AVR is used 'as is'.
 *
 * \section install_sec Hardware
 * To run the software, it must be compiled according to your hardware.
 * See gpib.h for the assignments of AVR port lines to GPIB interface lines.
 * You can change the defines in gpib.h according to your hardware.
 * I used for testing the AVR board from pollin priced ca. 15 Euro. Every other board
 * is possible as long as you have enough port lines available. 16 ports are required.
 * 
 * \section GPIB bus setup
 * Set up GPIB bus addresses for your devices. The setup is done in/at the device. Check device
 * manual.
 * 
 * \section Ordner of switching on devices on bus
 * This is a little bit tricky. Best order is to switch on controller, and after that switch on the
 * devices.
 *
 * The software assumes two devices with addresses
 * 0x01 and 0x02. The controller itself has address 0x00. You can change number of devices
 * and assumed addresses in function gpib_controller_assign().
 * 
 * \section Software
 * This code implements a kind of a GPIB controller. It understands a few built in commands.
 * The hardware should be connected via GPIB interface, the controlling computer/terminal via RS232.
 * Then the following commands can be used:
 * \li .a <n> - set partner device address to value <n>. <n> is the GPIB device address, e.g. 3.
 * The partner device is the device to control.
 * \li .h - print some help.
 * \li .i - print out status of some GPIB lines and current partner address.
 * 
 * When in normal input mode, device commands can be entered. For commands that will have an answer
 * from the device under control, the answer is print out to the terminal. The controller is also
 * able to do a serial poll.
 * The code was tested with a Tektronix 2432a oscilloscope and with a Tektronix 1241 logic analyzer.
 * The implementation may be specific to Tektronix and may not work with other devices.  
 * But it's worth a try.
 * 
 * \section Files
 * The implementation of GPIB protocol and controller functions itself is in gpib.h and gpib.c. 
 * The controller is implemented in main_gpib.c as a big never-ending loop.
 *
 * Errors are usually timeouts because the device behaves different
 * then the controller expected. The timeout functionality is implemented as 16 bit timer in 
 * timer16.h and timer16.c. For generic common defines the file defs.h is used.
 * The Doxygen file contains doxygen controls (automatic creation of documentation from source).
 * 
 * \section gpib_cmds Some arbitrary GPIB commands
 * Here are some commands for testing.
 * \subsection tek_2432a Tek 2432a
 * \li id?
 * \li INIT
 * \li MEASUREMENT WINDOW:ON
 * \li CURSOR FUNCTION: TIME,TARGET:CH1,UNITS:TIME:BASE 
 * \li CURSOR TPOS:ONE:200,TPOS:TWO:500
 * \li message 5:"a test message"
 * \li RUN?
 * \li CH1?
 * \li wavfrm?
 * \li curve?
 * \li wfmpre?
 * \li data encdg:ascii
 * 
 * \subsection tek_1241 Tek 1241
 * \li id?
 * \li init
 * \li dt?
 * \li acqmem?
 * 
 * \section Licence
 * This code can be used according to GNU General Public License.
 * 
 * \author 
 * Dennis Dingeldein
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "uart.h"
#include "gpib.h"
#include "timer16.h"

/** if WRITE is defined, the "real" controller code is used, if not defined, simple
 * listener code is executed. */
#define WRITE

#define ASCII_CODE_CR 0x0d
#define ASCII_CODE_LF 0x0a

#define ADDRESS_NOT_SET 0xff

uchar input_process(void);
void printHelp();
char *getRevision();

/** buffers used for commands and output strings */
uchar buf[80], cmd_buf[64];
/** pointer in buffer */
int buf_ptr = 0;

/** set to 1 to do line echo of all chars received by controller */
int rs232_remote_echo = 1;

/**
 * GPIB controller main function
 * \brief Implementation of GPIB controller. Reads a command from RS232, sends it via bus.
 * If The command contains a '?', an answer from the device is expected and read in. The
 * answer then is printed out. If an SRQ occured, a serial poll is initiated.
 * 
 */
int main(void) {
	uchar b, e;
	uchar partnerAddress = ADDRESS_NOT_SET; // 0xff means NO address assigned
	int old_time = 0;
	uchar srq;
	uchar is_query = 0;
	uchar command_ready = 0;
	char sbuf[32];
	uchar do_prompt = 1;

	/*
	 *  Initialize UART library, pass baudrate and avr cpu clock 
	 *  with the macro UART_BAUD_SELECT()
	 */DI();

	/*
	 * now enable interrupt, since UART and TIMER library is interrupt controlled
	 */sei();

	/** print some usage infos */
	printHelp();

#ifdef WRITE
	/*
	 * WRITE: Controller talks and listens
	 */

	// init timer for timeout detection
	timer16_init();

	// init gpib lines
	gpib_init();
	// init controller part - assign bus 
	gpib_controller_assign(0x00);

	/* controller loops forever:
	 * 1. try to read command from user
	 * 2. send user entered command, if available, to listeners (act as talker, set devices to listeners)
	 * 3. if command was a query, read the answer from device (become listener and set device to talker)
	 * 	4. check if SRQ occured and handle that
	 */
	for (;;) {

		if (do_prompt) {
			uart_puts("> ");
			do_prompt = 0;
		}
		// input processing via rs232
		// command_ready may already been set by SRQ that occured before
		if (!command_ready)
			command_ready = input_process();

		// check for internal command
		if (command_ready) {
			// all internal cmds start with a '.'
			if (buf[0] == '.') {
				uart_puts("\n\rInternal command: ");
				uart_puts((char*) buf);
				uart_puts("\n\r");
				// reset local vars for command string reading
				buf_ptr = 0;
				command_ready = 0;

				switch (buf[1]) {
				case 'a':
					/* set partner address */
					partnerAddress = atoi((char*) (&(buf[2])));
					sprintf(sbuf, "Set partner address to %u\n\r",
							partnerAddress);
					uart_puts(sbuf);
					gpib_set_partner(partnerAddress);
					break;
				case 'h':
					/* print some usage infos */
					printHelp();
					break;
				case 'i':
					sprintf(buf, "Partner address is: %u\n\r", partnerAddress);
					uart_puts(buf);
					gpib_info();
					break;
				default:
					uart_puts("unknown command\n\r");
					printHelp();
					break;
				}
				do_prompt = 1;
			}
		}

		if (command_ready == 1 && (partnerAddress == ADDRESS_NOT_SET)) {
			uart_puts(
					"\n\rDevice address is not set. Please set the device address before sending commands.");
			uart_puts("\n\r");
			command_ready = 0;
			buf_ptr = 0;
			do_prompt = 1;
		}

		// if a command was entered, send it to listeners
		if (command_ready) {
			// send UNT and UNL commands (unlisten and untalk)
			// effect: all talker stop talking and all listeners stop listening
			cmd_buf[0] = G_CMD_UNT;
			gpib_cmd(cmd_buf, 1);
			cmd_buf[0] = G_CMD_UNL;
			gpib_cmd(cmd_buf, 1);

			// set device (oszi) to listener mode
			partnerAddress = address2ListenerAddress(gpib_get_partner());
			cmd_buf[0] = partnerAddress;
			gpib_cmd(cmd_buf, 1);

			// set myself (controller) to talker mode
			partnerAddress = address2TalkerAddress(gpib_get_address());
			cmd_buf[0] = partnerAddress;
			gpib_cmd(cmd_buf, 1);

			// put out command to listeners
			uart_puts("\n\rcommand: ");
			uart_puts((char*) buf);
			uart_puts("\n\r");
			// gpib bus write
			gpib_write(buf, 0);

			// check if query or command only
			// all queries contain a '?'
			if (strchr((char*) buf, '?') != NULL) {
				uart_puts("Query. Will check for answer.\n\r");
				is_query = 1;
			} else {
				uart_puts("Command only.\n\r> ");
				is_query = 0;
			}

			// reset local vars for command string reading
			buf_ptr = 0;
			command_ready = 0;
		}

		// if we sent a query, read the answer
		if (is_query) {
			// UNT and UNL
			cmd_buf[0] = G_CMD_UNT;
			gpib_cmd(cmd_buf, 1);
			cmd_buf[0] = G_CMD_UNL;
			gpib_cmd(cmd_buf, 1);

			// set myself (controller) to listener mode
			partnerAddress = address2ListenerAddress(gpib_get_address());
			cmd_buf[0] = partnerAddress;
			gpib_cmd(cmd_buf, 1);

			// set device (oszi) to talker mode
			partnerAddress = address2TalkerAddress(gpib_get_partner());
			cmd_buf[0] = partnerAddress;
			gpib_cmd(cmd_buf, 1);

			// read the answer until EOI is detected (then e becomes true)
			do {
				// gpib bus receive
				e = gpib_receive(&b);
				// write out character
				uart_putc(b);
				//sprintf((char*)buf,"%02x - %c\n\r", b, b);
				//uart_puts((char*)buf);
			} while (!e);

			// send UNT and UNL commands (unlisten and untalk)
			// effect: all talker stop talking and all listeners stop listening
			cmd_buf[0] = G_CMD_UNT;
			gpib_cmd(cmd_buf, 1);
			cmd_buf[0] = G_CMD_UNL;
			gpib_cmd(cmd_buf, 1);

			uart_puts("\n\r"); // tek1241 is not sending cr,lf at command end, so create it always itself
			uart_puts("> ");
			// reset for next command
			is_query = 0;
		}

		// SRQ detection - do this every time when time value s has changed
		// s is incremented every second. So we check once a second.
		srq = 0;
		if (old_time == 0) {
			// old_time value initialization on first call with value s
			old_time = s;
		} else {
			if (s > old_time) {
				// some time has passed - check if srq was set
				srq = bit_is_clear(PIND,G_SRQ);
				if (srq)
					uart_puts("\n\rSRQ detected.\n\r");
			}
		}

		// SRQ handling by doing serial poll
		if (srq) {
			// reset srq for next call
			srq = 0;
			// handle srq with serial poll
			partnerAddress = gpib_serial_poll();
			gpib_set_partner(partnerAddress);
			// check status for reason
			buf[0] = 'E';
			buf[1] = 'V';
			buf[2] = 'E';
			buf[3] = 'N';
			buf[4] = 'T';
			buf[5] = '?';
			buf[6] = '\0';
			buf_ptr = 6;
			command_ready = 1;
		}
	}
#else
	/*
	 * READ: Controller=me is LISTENER ONLY
	 */

	// init gpib lines
	gpib_init();

	for(;;) {

		// 
		e = gpib_receive(&b);
		uart_putc(b);
		if (e)
		uart_puts("\n\rEOI\n\r");

		// input processing via rs232
		input_process();

	}
#endif
}

/**
 * Processing user input
 * \brief Read in user input via rs232 using peter fleurys UART library.
 * \returns The character read in
 */

uchar input_process(void) {
	unsigned int c;
	uchar ch;
	uchar ret = 0;

	/*
	 * Get received character from ringbuffer
	 * uart_getc() returns in the lower byte the received character and 
	 * in the higher byte (bitmask) the last receive error
	 * UART_NO_DATA is returned when no data is available.
	 *
	 */
	c = uart_getc();
	if (c & UART_NO_DATA) {
		// no data available from UART
		return 0;
	}

	/*
	 * new data available from UART
	 * check for Frame or Overrun error
	 */
	if (c & UART_FRAME_ERROR) {
		/* Framing Error detected, i.e no stop bit detected */
		uart_puts_P("UART Frame Error: ");
	}
	if (c & UART_OVERRUN_ERROR) {
		/* 
		 * Overrun, a character already present in the UART UDR register was 
		 * not read by the interrupt handler before the next character arrived,
		 * one or more received characters have been dropped
		 */
		uart_puts_P("UART Overrun Error: ");
	}
	if (c & UART_BUFFER_OVERFLOW) {
		/* 
		 * We are not reading the receive buffer fast enough,
		 * one or more received character have been dropped 
		 */
		uart_puts_P("Buffer overflow error: ");
	}

	/* 
	 * send received character back depending on global flag
	 */
	if (rs232_remote_echo)
		uart_putc((unsigned char) c);

	// make uchar from character in int value
	ch = (uchar) c;
	// add to buffer
	buf[buf_ptr++] = ch;
	// terminate string
	buf[buf_ptr] = '\0';

	// <CR> means command input is complete
	if (ch == ASCII_CODE_CR) {
		// adjust string terminator
		buf[--buf_ptr] = '\0';
		ret = 1;
	}

	return ret;
}

#define REVISION "0.7"

void printHelp() {
#ifdef WRITE
	sprintf(
			buf,
			"\n\rGPIB Controller (T/L/C) (Rev.%s) (c) spurtikus.de 2008-2015\n\r",
			REVISION);
	uart_puts(buf);
#else
	uart_puts("\n\rGPIB Listener Only (L) (Rev.%s) (c) spurtikus.de 2008-2015\n\r", REVISION);
	uart_puts(buf);
#endif
	uart_puts("Internal commands:\n\r");
	uart_puts(
			".a <device address> - set address of device to communicate with\n\r");
	uart_puts(".h - print help\n\r");
	uart_puts(".i - dump info about GPIB lines\n\r");
}

