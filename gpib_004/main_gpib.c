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

/** state machine states, see main() */
#define S_INITIAL 1
#define S_FIRST_BYTE_INT 2
#define S_FIRST_BYTE_GPIB 3
#define S_SEND_BYTES 4
#define S_GPIB_ANSWER 5
#define S_GPIB_NO_ANSWER 6
#define S_FINAL 7

/** set to 1 to do line echo of all chars received by controller */
uint8_t rs232_remote_echo = 1;
/** Xon/Xoff flow control flag */
uint8_t xonXoffMode = 1;
/** srq enabled mode */
uint8_t srq_enabled = 1;
/** if !=0 break lines received from gpib at that line position */
uint8_t linebreak = 80;

#define COMMAND_INPUT_BUFFER_SIZE 80

uchar input_process(uchar *buf, int *ptr);
void printHelp();
void handle_internal_commands(uchar *cmd);
void receiveAnswer();

#define ARB_TEST
#ifdef ARB_TEST

void arb_ramp() {
	uchar b[10];

	gpib_prepare_write();
	gpib_write_prologue(0);

	gpib_write_string("SOUR:LIST:SEGM:VOLT ");

	for (int i = 0; i < 4096; i++) {
		int f = i / 1000;
		sprintf(b, "%d", f);
		gpib_write_byte(b[0], 0);
		if (i < 4096 - 1) {
			gpib_write_byte(',', 0);
		}
	}

	gpib_write_byte(ASCII_CODE_CR, 1);
	gpib_write_epilogue(0);
	gpib_untalkUnlisten();
}

void arb() {
	gpib_write_command("*RST");
	gpib_write_command("SOUR:ROSC:SOUR INT;");
	gpib_write_command(":SOUR:FREQ:FIX 1E3;");
	gpib_write_command(":SOUR:FUNC:SHAP USER;");
	gpib_write_command(":SOUR:VOLT:LEV:IMM:AMPL 5V");
	gpib_write_command("SOUR:LIST:SEGM:SEL A"); // no ';' at end!
	arb_ramp();
	gpib_write_command("SOUR:FUNC:USER A");
	gpib_write_command("INIT:IMM");
	//send_command("SOUR:LIST:SEGM:SEL?");
}
#endif

/**
 * Read two integers from string like "45 56" or one integer. In latter case
 * the second integer is initialized with a special value.
 */
static void stringToTwoUchars(char *string, uchar *a, uchar *b) {
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
 * Checks for errors.
 *
 * Reads error queue. output is one error per line.
 *
 */
void check_errors() {
	char *error_cmd = "SYST:ERR?";
	uchar msg[80];
	uchar b, e;
	uchar colptr = 0;
	uchar allErrorsRead = 0;

	while (!allErrorsRead) {
		gpib_write_command(error_cmd);
		gpib_prepare_read();
		// read the answer until EOI is detected (then e becomes true)
		uchar i = 0;
		do {
			// gpib bus receive
			e = gpib_receive(&b);
			msg[i++] = b;
		} while (!e);
		// terminate string
		//msg[i++] = '\n';
		msg[i++] = '\r';
		msg[i] = 0x00;
		// check if all errors have been read
		if (strncmp(msg,"+0,",3)==0) {
			allErrorsRead=1;
		} else {
			uart_puts((char*) msg);
		}
	}
}

/**
 * Reads in character into parameter c. Checks for errors and prints them out.
 * Returns 0 if there is no char to read, 1 if there was a char read in.
 */
uchar input_char(uchar *ch) {
	unsigned int c;
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
	// make uchar from character in int value
	*ch = (uchar) c;

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
	return 1;
}

/**
 * Process input char.
 * Echo if required, add char to buffer if possible.
 * If a CR is read command end is suspected.
 * If buffer is full then the following happens:
 * a) xon/xoff mode forward buffer to GPIB
 * b) no flow control: prints error message that input buffer is full.
 *
 * Returns 1 if command end is detected, 0 otherwise.
 */
uchar process_char(uchar *buf, uchar ch, int *ptr) {
	uchar ret = 0;
	/*
	 * send received character back depending on global flag
	 */
	if (rs232_remote_echo) {
		uart_putc((unsigned char) ch);
	}

	// if input buffer is not full, add char
	if (*ptr < COMMAND_INPUT_BUFFER_SIZE - 1) {
		buf[(*ptr)++] = ch;
		buf[*ptr] = '\0';
	}

	// if command ends or buffer is full ...
	if (ch == ASCII_CODE_CR || *ptr >= COMMAND_INPUT_BUFFER_SIZE - 1) {
		if (ch == ASCII_CODE_CR) {
			// adjust string terminator
			buf[--(*ptr)] = '\0';
			// let calling function send last command part (or command itself)
			ret = 1;
		} else {
			// send intermediate part of command.
			uart_puts_P("Command overflow.");
			*ptr = 0;
		}
	}
	return ret;
}

/**
 * Processing user input
 * \brief Read in user input via rs232 using peter fleurys UART library.
 *
 * Idea for code below: if xon/xoff flow control, we assume large command line.
 * then stay in this function, get all chars in, send them with gpib_write() in parts.
 *
 * If we get last part (CR received) set ret to one and return. Then main() will send the
 * last part.
 * This approach handles small single line commands (needing no flow control) and large
 * multi-line commands if flow control is xon/xoff.
 *
 * \returns The character read in
 */
uchar input_process(uchar *buf, int *ptr) {
	uchar ch, ret = 0;

	if (uart_get_flow_control() == FLOWCONTROL_XONXOFF) {
		while (!ret) {
			// if nothing can be read in, return
			if (!input_char(&ch)) {
				return 0;
			}
			ret = process_char(buf, ch, ptr);
		}
	} else {
		// if nothing can be read in, return
		if (!input_char(&ch)) {
			return 0;
		}
		ret = process_char(buf, ch, ptr);
	}
	return ret;
}

/**
 * Handles builtin commands.
 */
void handle_internal_commands(uchar *cmd) {
	uchar val, val1;

	switch (cmd[1]) {
	case 'a':
		/* set partner primary+secondary address */
		stringToTwoUchars((char*) (&(cmd[2])), &val, &val1);
		sprintf(cmd, "Set partner address, primary: %u , secondary: %u\n\r",
				val, val1);
		uart_puts(cmd);
		gpib_set_partner_address(val, val1);
		break;
	case 's':
		/* set partner secondary address */
		val = atoi((char*) (&(cmd[2])));
		sprintf(cmd, "Set partner secondary address to %u\n\r", val);
		uart_puts(cmd);
		gpib_set_partner_secondary(val);
		break;
	case '+':
		/* add device */
		stringToTwoUchars((char*) (&(cmd[2])), &val, &val1);
		sprintf(cmd, "Add device, primary: %u , secondary: %u\n\r", val, val1);
		uart_puts(cmd);
		gpib_add_partner_address(val, val1);
		break;
	case '-':
		/* add device */
		stringToTwoUchars((char*) (&(cmd[2])), &val, &val1);
		sprintf(cmd, "Remove device, primary: %u , secondary: %u\n\r", val,
				val1);
		uart_puts(cmd);
		gpib_remove_partner_address(val, val1);
		break;
	case 'x':
		/* Xon/Xoff flow control */
		if (!xonXoffMode) {
			xonXoffMode = 1;
			uart_set_flow_control(FLOWCONTROL_XONXOFF);
			uart_puts_P("xon/xoff flowcontrol on\n\r");
		} else {
			xonXoffMode = 0;
			uart_set_flow_control(FLOWCONTROL_NONE);
			uart_puts_P("xon/xoff flowcontrol off\n\r");
		}
		break;
	case 'h':
		/* print some usage infos */
		printHelp();
		break;
	case 'i':
		gpib_info();
		sprintf(cmd, "Xon/Xoff flow control: %u\n\r", xonXoffMode);
		uart_puts(cmd);
		break;
	case 'e':
		uart_puts_P("Check errors\n\r");
		check_errors();
		break;
#ifdef ARB_TEST
	case 'z':
		uart_puts("arb\n\r");
		arb();
		uart_puts("arb done\n\r");
		break;
#endif
	default:
		uart_puts_P("unknown command\n\r");
		printHelp();
		break;
	}
}

/**
 * Receives answer after command was sent.
 */
void receiveAnswer() {
	uchar b, e;
	uchar colptr = 0;

	gpib_prepare_read();
	// read the answer until EOI is detected (then e becomes true)
	do {
		// gpib bus receive
		e = gpib_receive(&b);
		// write out character
		uart_putc(b);
		if (linebreak && (colptr++ == linebreak)) {
			uart_puts_P("\n\r");
			colptr = 0;
		}
		//sprintf((char*)buf,"%02x - %c\n\r", b, b);
		//uart_puts((char*)buf);
	} while (!e);
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
				uart_puts_P("\n\rSRQ detected.\n\r");
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
		uart_puts_P(
				"\n\rSRQ emitter is not in list of known devices. SRQ Ignored.\n\r");
		uart_puts_P("\n\rSRQs are disabled now.\n\r");
		srq_enabled = 0;
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

void printHelp() {
	char buf[COMMAND_INPUT_BUFFER_SIZE];
	sprintf(buf, "\n\rGPIB Controller (Rev.%s) (c) spurtikus.de 2008-2015\n\r",
	REVISION);
	uart_puts(buf);
	uart_puts_P("Internal commands:\n\r");
	uart_puts(
			".a <primary> [<secondary>] - set prim./second. address of remote device\n\r");
	uart_puts_P(".s <secondary> - set secondary address of remote device\n\r");
	uart_puts_P(
			".+ <n> - add partner device address to list of known devices.\n\r");
	uart_puts_P(
			".- <n> - remove partner device address from list of known devices.\n\r");
	uart_puts_P(".x - toggle Xon/Xoff flow control.\n\r");
	uart_puts_P(".h - print help.\n\r");
	uart_puts_P(".i - dump info about GPIB lines.\n\r");
}

/**
 * State machine.
 *
 * 1. try to read command from user
 * 2. send user entered command, if available, to listeners (act as talker, set devices to listeners)
 * 3. if command was a query, read the answer from device (become listener and set device to talker)
 * 4. check if SRQ occured and handle that
 *
 */
void state_machine() {
	int old_time = 0;
	uchar is_query = 0;
	uchar do_prompt = 1;
	uchar ch;

	/** buffers used for commands and output strings */
	uchar buf[COMMAND_INPUT_BUFFER_SIZE];
	/** pointer in buffer */
	int buf_ptr = 0;

	uchar state = S_INITIAL;
	for (;;) {
		if (state == S_INITIAL) {
			if (do_prompt) {
				uart_puts("> ");
				do_prompt = 0;
				is_query = 0;
			}
		}

		if (!input_char(&ch))
			continue;

		// byte received. Decide with state what to do.

		if (state == S_INITIAL) {
			// internal or external command?
			if (ch == '.') {
				// internal command
				state = S_FIRST_BYTE_INT;
			} else {
				// gpib command
				state = S_FIRST_BYTE_GPIB;

			}
		}

		if (state == S_FIRST_BYTE_INT) {
			buf[0] = ch;
			buf_ptr = 1;
			// send received character back depending on global flag
			if (rs232_remote_echo) {
				uart_putc((unsigned char) ch);
			}
			// collect line until CR
			while (!input_process(buf, &buf_ptr))
				;
			uart_puts_P("\n\r");
			// execute internal command
			handle_internal_commands(buf);
			// reset local vars
			state = S_INITIAL;
			buf_ptr = 0;
			do_prompt = 1;
		}

		if (state == S_FIRST_BYTE_GPIB) {
			// GPIB command. Check if a partner was defined.
			if (gpib_get_partner_pad() == ADDRESS_NOT_SET) {
				uart_puts_P(
						"Device address is not set. Can not send command.\n\r");
				// reset local vars
				state = S_INITIAL;
				buf_ptr = 0;
				do_prompt = 1;
				is_query = 0;
			} else {
				// write prologue
				state = S_SEND_BYTES;
				gpib_prepare_write();
				gpib_write_prologue(0);
			}
		}

		if (state == S_SEND_BYTES) {
			// send received character back depending on global flag
			if (rs232_remote_echo) {
				uart_putc((unsigned char) ch);
			}
			uchar isLastByte = (ch == ASCII_CODE_CR);
			gpib_write_byte(ch, isLastByte);

			if (ch == '?') {
				is_query = 1;
			}

			if (isLastByte) {
				uart_puts_P("\n\r");
				if (is_query) {
					//uart_puts("Query. Will check for answer.\n\r");
					state = S_GPIB_ANSWER;
				} else {
					//uart_puts("Command only.\n\r");
					state = S_GPIB_NO_ANSWER;
				}
			}
		}

		// write epilogue
		if (state == S_GPIB_ANSWER || state == S_GPIB_NO_ANSWER) {
			gpib_write_epilogue(0);
			buf_ptr = 0;
		}

		// if we sent a query, read the answer
		if (state == S_GPIB_ANSWER) {
			receiveAnswer();
			state = S_FINAL;
		}

		// finalize state machine
		if (state == S_GPIB_NO_ANSWER || state == S_FINAL) {
			// untalk/unlisten all partners
			gpib_untalkUnlisten();
			// some devices do not send cr,lf at command end, so create it always itself
			uart_puts_P("\n\r");
			do_prompt = 1;
			state = S_INITIAL;
		}

		// SRQ detection - do this every time when time value s has changed
		// s is incremented every second. So we check once a second.
		if (srq_enabled && srq_occured(&old_time)) {
			// TODO: make handle srq work again
			// the returned command_ready was interpreted to read in an answer
			// but this was turned of for new input loop
			// next two lines replace that but must be tested.
			if (handle_srq(buf, &buf_ptr)) {
				state = S_GPIB_ANSWER;
			}
		}
	}
}

/**
 * GPIB controller main function
 * \brief Implementation of GPIB controller. Reads a command from RS232, sends it via bus.
 * If The command contains a '?', an answer from the device is expected and read in. The
 * answer then is printed out. If an SRQ occured, a serial poll is initiated.
 * 
 */
int main(void) {
	/*
	 *  Initialize UART library, pass baudrate and avr cpu clock 
	 *  with the macro UART_BAUD_SELECT()
	 */DI();

	/*
	 * now enable interrupt, since UART and TIMER library is interrupt controlled
	 */sei();

	/** print some usage infos */
	printHelp();

	// init timer for timeout detection
	timer16_init();

	// init gpib lines
	gpib_init();
	// init controller part - assign bus 
	gpib_controller_assign(0x00);

	if (xonXoffMode) {
		uart_set_flow_control(FLOWCONTROL_XONXOFF);
	}

	// state machine loops forever
	state_machine();
}
