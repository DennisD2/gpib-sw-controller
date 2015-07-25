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
 * I used for testing the AVR board from Pollin priced ca. 15 Euro. Every other board
 * is possible as long as you have enough port lines available. 16 ports are required.
 * 
 * \section GPIB bus setup
 * Set up GPIB bus addresses for your devices. The setup is done in/at the device. Check device
 * manual. The controller can handle primary only and primary/secondary addresses.
 * 
 * \section Order of switching on devices on bus
 * This is a little bit tricky. Best order is to switch on controller, and after that switch on the
 * devices.
 *
 * The software keeps a list of "known devices". These are devices that are included in a
 * serial poll (after a SRQ). You must add devices manually to the list. If a device not in the
 * list generates a SRQ this SRQ is not correctly handled.
 * 
 * \section Software
 * This code implements a kind of a GPIB controller. It understands a few built in commands.
 * The hardware should be connected via GPIB interface, the controlling computer/terminal via RS232.
 * Then the following commands can be used. <n> is a GPIB device address, e.g. 3.:
 * \li .a <n> - set partner device address to value <n>.
 * The partner device is the device to control.
 * \li .+ <n> - add partner device address to list of known devices. Used for SRQ handling.
 * \li .- <n> - remove partner device address from list of known devices. Used for SRQ handling.
 * \li .h - print some help.
 * \li .i - print out status of some GPIB lines and current partner address.
 * 
 * When in normal input mode, device commands can be entered. For commands that will have an answer
 * from the device under control, the answer is print out to the terminal. The controller is also
 * able to do a serial poll.
 * The code was tested with the following devices:
 *
 * \li Tektronix 2432a oscilloscope
 * \li Tektronix 1241 logic analyzer
 * \li HP3478 multimeter
 * \li HP75000 VXI mainframe with several modules installed (uses secondary addresses)
 *
 * The implementation may not work well with your device.But it's worth a try :-)
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
 * \subsection HP3768
 * \li H1?
 *
 * \subsection HP75000 / E1300 board using secondary address feature
 * \li *rst
 * \li *idn?
 * \li SYST:TIME?
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

uchar input_process(void);
void printHelp();
void handle_internal_commands(uchar *commandString);

/** buffers used for commands and output strings */
uchar buf[80];
/** pointer in buffer */
int buf_ptr = 0;

/** set to 1 to do line echo of all chars received by controller */
uint8_t rs232_remote_echo = 1;
/** Xon/Xoff flow control flag */
uint8_t xonXoffMode = 0;

/**
 * Read two integers from string like "45 56" or one integer. In latter case
 * the second integer is initialized with a special value.
 */
void stringToTwoUchars(char *string, uchar *a, uchar *b) {
	char *token = strtok(string, " ");
	*a = atoi((char*) token);
	token = strtok(NULL, " ");
	if (token != NULL) {
		*b = atoi((char*) token);
	} else {
		*b = ADDRESS_NOT_SET;
	}
}

/**
 * Handles builtin commands.
 */
void handle_internal_commands(uchar *commandString) {
	uchar sbuf[64];
	uchar val, val1;

	switch (buf[1]) {
	case 'a':
		/* set partner primary+secondary address */
		stringToTwoUchars((char*) (&(buf[2])), &val, &val1);
		sprintf(sbuf, "Set partner address, primary: %u , secondary: %u\n\r",
				val, val1);
		uart_puts(sbuf);
		gpib_set_partner_address(val, val1);
		break;
	case 's':
		/* set partner secondary address */
		val = atoi((char*) (&(buf[2])));
		sprintf(sbuf, "Set partner secondary address to %u\n\r", val);
		uart_puts(sbuf);
		gpib_set_partner_secondary(val);
		break;
	case '+':
		/* add device */
		stringToTwoUchars((char*) (&(buf[2])), &val, &val1);
		sprintf(sbuf, "Add device, primary: %u , secondary: %u\n\r", val, val1);
		uart_puts(sbuf);
		gpib_add_partner_address(val, val1);
		break;
	case '-':
		/* add device */
		stringToTwoUchars((char*) (&(buf[2])), &val, &val1);
		sprintf(sbuf, "Remove device, primary: %u , secondary: %u\n\r", val,
				val1);
		uart_puts(sbuf);
		gpib_remove_partner_address(val, val1);
		break;
	case 'x':
		/* Xon/Xoff flow control */
		if (!xonXoffMode) {
			xonXoffMode = 1;
			uart_set_flow_control(FLOWCONTROL_XONXOFF);
			uart_puts("xon/xoff flowcontrol on");
		} else {
			xonXoffMode = 0;
			uart_set_flow_control(FLOWCONTROL_NONE);
			uart_puts("xon/xoff flowcontrol off");
		}
		break;
	case 'h':
		/* print some usage infos */
		printHelp();
		break;
	case 'i':
		gpib_info();
		sprintf(sbuf, "Xon/Xoff flow control: %u\n\r", xonXoffMode);
		uart_puts(sbuf);
		break;
	default:
		uart_puts("unknown command\n\r");
		printHelp();
		break;
	}
}

/**
 * Sends a command.
 *
 * Returns 1 if command is a query, 0 otherwise.
 */
uchar send_command(uchar *commandString) {
	uchar controlString[8];
	uchar is_query;

	// send UNT and UNL commands (unlisten and untalk)
	// effect: all talker stop talking and all listeners stop listening
	controlString[0] = G_CMD_UNT;
	gpib_cmd(controlString, 1);
	controlString[0] = G_CMD_UNL;
	gpib_cmd(controlString, 1);

	// set device to listener mode
	controlString[0] = address2ListenerAddress(gpib_get_partner_pad());
	gpib_cmd(controlString, 1);
	// send secondary address if required
	if (gpib_get_partner_sad() != ADDRESS_NOT_SET) {
		controlString[0] = secondaryAdressToAdressByte(gpib_get_partner_sad());
		gpib_cmd(controlString, 1);
	}

	// set myself (controller) to talker mode
	controlString[0] = address2TalkerAddress(gpib_get_address());
	gpib_cmd(controlString, 1);

	//uart_puts("\n\rcommand: ");
	//uart_puts((char*) commandString);
	//uart_puts("\n\r");
	// gpib bus write
	// put out command to listeners
	gpib_write(commandString, 0);

	// check if query or command only
	if (strchr((char*) commandString, '?') != NULL) {
		//uart_puts("Query. Will check for answer.\n\r");
		is_query = 1;
	} else {
		//uart_puts("Command only.\n\r");
		is_query = 0;
	}
	return is_query;
}

/**
 * Receives answer after command was sent.
 */
void receiveAnswer() {
	uchar controlString[8];
	uchar b, e;

	// UNT and UNL
	controlString[0] = G_CMD_UNT;
	gpib_cmd(controlString, 1);
	controlString[0] = G_CMD_UNL;
	gpib_cmd(controlString, 1);

	// set myself (controller) to listener mode
	controlString[0] = address2ListenerAddress(gpib_get_address());
	gpib_cmd(controlString, 1);

	// set device to talker mode
	controlString[0] = address2TalkerAddress(gpib_get_partner_pad());
	gpib_cmd(controlString, 1);
	// secondary address if required
	if (gpib_get_partner_sad() != ADDRESS_NOT_SET) {
		controlString[0] = secondaryAdressToAdressByte(gpib_get_partner_sad());
		gpib_cmd(controlString, 1);
	}

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
	controlString[0] = G_CMD_UNT;
	gpib_cmd(controlString, 1);
	controlString[0] = G_CMD_UNL;
	gpib_cmd(controlString, 1);
}

/**
 * Check if a SRQ occured
 */
uchar srq_occured(int* old_time) {
	uchar srq = 0;
	if (*old_time == 0) {
		// old_time value initialization on first call with value s
		*old_time = s;
	} else {
		if (s > *old_time) {
			// some time has passed - check if srq was set
			srq = bit_is_clear(PIND, G_SRQ);
			if (srq)
				uart_puts("\n\rSRQ detected.\n\r");
		}
	}
	return srq;
}

/**
 * Handles SRQs by doing serial poll
 *
 */
uchar handle_srq(uchar *buf, int *buf_ptr) {
	uchar command_ready = 0;
	uint8_t primary, secondary;

	if (!gpib_serial_poll(&primary, &secondary)) {
		uart_puts(
				"\n\rSRQ emitter is not in list of known devices. SRQ Ignored.\n\r");
		return command_ready;
	}
	gpib_set_partner_address(primary, secondary);

	if (gpib_get_flavour() == FLAVOUR_TEK) {
		// Tek: check status for reason
		buf[0] = 'E';
		buf[1] = 'V';
		buf[2] = 'E';
		buf[3] = 'N';
		buf[4] = 'T';
		buf[5] = '?';
		buf[6] = '\0';
		*buf_ptr = 6;
		command_ready = 1;
	}
	return command_ready;
}

/**
 * GPIB controller main function
 * \brief Implementation of GPIB controller. Reads a command from RS232, sends it via bus.
 * If The command contains a '?', an answer from the device is expected and read in. The
 * answer then is printed out. If an SRQ occured, a serial poll is initiated.
 * 
 */
int main(void) {
	int old_time = 0;
	uchar is_query = 0;
	uchar command_ready = 0;
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
		// command_ready may already been set by SRQ that occurred before
		if (!command_ready)
			command_ready = input_process();

		if (command_ready) {
			uart_puts("\n\r");
		}

		// check for internal commands
		if (command_ready && buf[0] == '.') {
			// all internal cmds start with a '.'
			//uart_puts("\n\rInternal command: ");
			//uart_puts((char*) buf);
			//uart_puts("\n\r");
			handle_internal_commands(buf);
			// reset local vars for command string reading
			buf_ptr = 0;
			command_ready = 0;
			do_prompt = 1;
			is_query = 0;
		}

		// GPIB command. Check if a partner was defined.
		if (command_ready && (gpib_get_partner_pad() == ADDRESS_NOT_SET)) {
			uart_puts("Device address is not set. Will not send commands.\n\r");
			// reset local vars for command string reading
			command_ready = 0;
			buf_ptr = 0;
			do_prompt = 1;
		}

		// GPIB command and valid partner. Send the command.
		if (command_ready) {
			//uart_puts("\n\rGPIB command: ");
			//uart_puts((char*) buf);
			//uart_puts("\n\r");
			is_query = send_command(buf);
			// reset local vars for command string reading
			command_ready = 0;
			buf_ptr = 0;
			do_prompt = 1;
		}

		// if we sent a query, read the answer
		if (is_query) {
			receiveAnswer();
			// reset for next command
			is_query = 0;
			// some devices do not send cr,lf at command end, so create it always itself
			uart_puts("\n\r");
			do_prompt = 1;
		}

		// SRQ detection - do this every time when time value s has changed
		// s is incremented every second. So we check once a second.
		if (srq_occured(&old_time)) {
			command_ready = handle_srq(buf, &buf_ptr);
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
	sprintf(buf,
			"\n\rGPIB Controller (T/L/C) (Rev.%s) (c) spurtikus.de 2008-2015\n\r",
			REVISION);
	uart_puts(buf);
#else
	uart_puts("\n\rGPIB Listener Only (L) (Rev.%s) (c) spurtikus.de 2008-2015\n\r", REVISION);
	uart_puts(buf);
#endif
	uart_puts("Internal commands:\n\r");
	uart_puts(
			".a <primary> [<secondary>] - set primary/secondary address of remote device\n\r");
	uart_puts(".s <secondary> - set secondary address of of remote device\n\r");
	uart_puts(
			".+ <n> - add partner device address to list of known devices.\n\r");
	uart_puts(
			".- <n> - remove partner device address from list of known devices.\n\r");
	uart_puts(".x - toggle Xon/Xoff flow control.\n\r");
	uart_puts(".h - print help.\n\r");
	uart_puts(".i - dump info about GPIB lines.\n\r");
}

