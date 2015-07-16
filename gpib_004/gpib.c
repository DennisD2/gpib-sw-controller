/*************************************************************************
 Author:   	$Author: dennis $
 File:     	$HeadURL: file://localhost/home/dennis/svn-store/avr-source/gpib_004/gpib.c $
 Date:  		$Date: 2012-04-19 16:33:45 +0200 (Do, 19 Apr 2012) $
 Revision: 	$Revision: 696 $
 Id: 		$Id: gpib.c 696 2012-04-19 14:33:45Z dennis $
 Licence:	GNU General Public License
 *************************************************************************/

/** 
 * \defgroup gpib_lib GPIB Library
 * \file gpib.c
 * \brief This file contains the implementation of the GPIB functionality.
 */

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "uart.h"

#include "gpib.h"
#include "timer16.h"

/**
 * if set, WITH_TIMEOUT means that waiting loops are interrupted by timeout.
 * if not set, code stays in waiting loop maybe forever.
 */
#define WITH_TIMEOUT

/** address type */
typedef struct {
	uchar primary;
	uchar secondary;
} address_t;

/** Controller object data */
typedef struct {
	uchar myaddress; /**< controller address,usually 0x00 */
	address_t partner; /**< currently addressed partner device */
	uchar talks; /**< true while controller is talker */
	uchar flavour; /**< controller flavour */
	address_t partners[MAX_PARTNER]; /**< list of active partners */
} gpib_controller_t;

/** controller object. Not to be accessed outside gpib.c */
static gpib_controller_t controller;

uchar _gpib_write(uchar *bytes, int length, uchar attention);

/**
 * Open Collector bit handling.
 * release : means set to HIGH.
 * If not open collector, we would use:
 *			   bit is output, set bit to 1
 *             #define release_bit(p,b) p |= _BV(b)
 *			   bit is output, set bit to 0
 */
#define release_bit(d,p,b) d &= ~_BV(b); p |= _BV(b);

/**
 * Open Collector bit handling.
 * assign: means set to LOW.
 * If not open collector, we would use:
 *				bit is output, set bit to 1
 *          	#define assign_bit(p,b)  p &= ~_BV(b)
 *				bit to input, switch pullup on
 *				bit is unknown, set bit to 0, bit to output, set bit to 0
 */
#define assign_bit(d,p,b)  p &= ~_BV(b); d |= _BV(b); p &= ~_BV(b);

/** buffer used for sending commands and print out strings */
uchar cmd_buf[100];

/**
 * Some basic delay function
 */
void delay_ms(unsigned short ms) {
	unsigned short outer1, outer2;
	outer1 = 200 * 12;

	while (outer1) {
		outer2 = 100;
		while (outer2) {
			while (ms)
				ms--;
			outer2--;
		}
		outer1--;
	}
}

/**
 * Init GPIB pins and variables.
 * \brief All signal lines not related to the controller part
 * 		are initialized with useful values.
 *		(The controller part initialization is done on gpib_controller_assert())
 */
void gpib_init(void) {
	// data lines - complete port A as input
	DDRA = 0x00;

	// handshake lines - on port D , everything as input
	DDRD &= ~_BV(G_DAV); // DAV 
	DDRD &= ~_BV(G_EOI); // EOI 
	DDRD &= ~_BV(G_SRQ); // SRQ 
	DDRD &= ~_BV(G_ATN); // ATN  
	DDRB &= ~_BV(G_REN); // REN  
	DDRB &= ~_BV(G_IFC); // IFC  

	// init handshake lines
	assign_bit(DDRD, PORTD, G_NRFD);
	// not ready for data now
	release_bit(DDRD, PORTD, G_NDAC);
	// initially: ok so far
}

/**
 * Receive a character from GPIB Bus.
 * \brief Does busy waiting until timeout value is reached.
 *  \param _byte 	Pointer to single character; the function stores herein the character read.
 * 		When errors occur during the function, the content of the parameter (i.e. *_byte) is undefined.
 *  \returns		On any error, 0xff is returned. in this case, the value of parameter *_byte is undefined.
 * 		Otherwise the value of the EOI signal line during read is returned. If EOI was assigned, a 0x01 is
 * 		returned. If EOI was not assigned, a 0x00 is returned. Assignment of EOI means that the talker
 * 		is sending the last character for this transmission.
 */
uchar gpib_receive(uchar* _byte) {
	int timeout;
	uchar byte, eoi;

	//uart_puts("\n\rgpib_receive()\n\r");

	if (controller.talks == 1) {
		*_byte = 0xff;
		return 0xff;
	}

	// handshake: set nrfd, means i am ready to receive some data
	release_bit(DDRD, PORTD, G_NRFD);
	assign_bit(DDRD, PORTD, G_NDAC);

	//gpib_info();

#ifdef WITH_TIMEOUT
	timeout = s + 5;
	//gpib_info();
	while ((PIND & _BV(G_DAV)) && (s <= timeout)) {
		if (s == timeout) {
			uart_puts("\n\rError: DAV timeout (1)\n\r");
			return 0xff;
		}
	}
#else
	loop_until_bit_is_clear(PIND,G_DAV);
#endif

	// handshake: clear NRFD, means i am busy now to read data
	assign_bit(DDRD, PORTD, G_NRFD);
	// read data
	byte = PINA ^ 0xff;

	// handshake: set ndac, means i have completed/accepted the read
	release_bit(DDRD, PORTD, G_NDAC);

#ifdef WITH_TIMEOUT
	timeout = s + 5;
	//gpib_info();
	while (!(PIND & _BV(G_DAV)) && (s <= timeout)) {
		if (s == timeout) {
			uart_puts("\n\rError: DAV timeout (2)\n\r");
			return 0xff;
		}
	}
#else
	loop_until_bit_is_set(PIND,G_DAV);
#endif
	// handshake: clear ndac (this is a prerequisite for receive next byte)
	assign_bit(DDRD, PORTD, G_NDAC);

	// check if last byte of transmission
	eoi = bit_is_clear(PIND, G_EOI);

	*_byte = byte;

	return eoi;
}

/**
 * Assign bus to me
 * \brief Initialization of the controller part.
 * 
 * \warning This function is not fully generic. I assume that two devices with address 0x01 and 0x02
 * are on the bus and fill the partners-array according to that assumption.
 * You have to change the initialization of the partner array according to your environment.
 * The partners-array is used by gpib_serial_poll() for looping over all existing devices.
 * 
 * \param address the address to be used by the controller. Usually 0x00.
 */
void gpib_controller_assign(uchar address) {
	controller.myaddress = address;
	controller.talks = 0;
	controller.partner.primary = ADDRESS_NOT_SET; // init default active partner
	controller.partner.secondary = ADDRESS_NOT_SET;
	controller.flavour = FLAVOUR_NONE;
	/** clear list of partners */
	gpib_clear_partners();
	// set up initial state of bus
	assign_bit(DDRB, PORTB, G_IFC);
	delay_ms(200);
	release_bit(DDRB, PORTB, G_IFC);
	// set up all devices for remote control
	assign_bit(DDRB, PORTB, G_REN);

	// DCL - device clear for all devices on bus
	cmd_buf[0] = G_CMD_DCL;
	gpib_cmd(cmd_buf, 1);
}

/**
 * Release bus 
 */
void gpib_controller_release(void) {
	// set up initial state of bus
	assign_bit(DDRB, PORTB, G_IFC);
	delay_ms(200);
	release_bit(DDRB, PORTB, G_IFC);
	// set up all devices for local control
	release_bit(DDRB, PORTB, G_REN);
}

/**
 * Write GPIB string to bus
 * \brief See _gpib_write() for further information.
 */
uchar gpib_write(uchar *bytes, int length) {
	// set attention arg false for ordinary strings
	return _gpib_write(bytes, length, (uchar) 0);
}

/**
 * Write GPIB command to bus
 * \brief See _gpib_write() for further information.
 */
uchar gpib_cmd(uchar *bytes, int length) {
	// set attention arg true for commands
	return _gpib_write(bytes, length, (uchar) 1);
}

/**
 * Write byte array to the bus.
 * \brief Precondition: Device must be allowed to talk. For a controller, this means all
 * other devices have been set to be listeners.
 * \param bytes byte array containing bytes to be send
 * \param length length of valid bytes in byte array or zero.
 * for binary data, lenght must be defined. For strings, length can be set to zero. Then the length of
 * the string is calculated internally.
 * \param attention attention tur means assign ATN signal line during write.
 */
uchar _gpib_write(uchar *bytes, int length, uchar attention) {
	uchar c;
	int i;
	int timeout;
	//uchar buf[64];

	// set talks state. This is used by ISR to recognize own talk
	// (controller must not talk to itself and must not take part in listener handshake when talking)
	controller.talks = 1;

	if (attention) {
		//uart_puts("\n\rgpib_controller_write()\n\r");
		// assign ATN for commands
		assign_bit(DDRD, PORTD, G_ATN);
	}

	if (length == 0) {
		// length==0 means this is a common C string, null-terminated.
		// then, length can be easily calculated
		length = strlen((char*) bytes);
	}
	// for debugging print out
	//	bytes[length]=0x00;
	//	if (length>1)
	//		sprintf( buf, "gpib_write: %s\n\r", (char *)bytes );
	//	else 
	//		sprintf( buf, "gpib_write: 0x%02x\n\r", bytes[0] );
	//	uart_puts((char*)buf);

	// release EOI during transmission
	release_bit(DDRD, PORTD, G_EOI);
	// release DAV, data not valid anymore
	release_bit(DDRD, PORTD, G_DAV);
	// release NRFD (just to be sure)
	release_bit(DDRD, PORTD, G_NRFD);

	// bytes[0] = 'a'
	// bytes[1] = 'b'
	// length = 2
	for (i = 0; i < length; i++) {

		// put data on bus
		c = bytes[i];
		//sprintf( buf, "char: %c\n\r", c );
		//uart_puts(buf);		

		release_bit(DDRD, PORTD, G_NDAC);
		//uart_puts("0");
		// wait for NDAC assign from all listeners
#ifdef WITH_TIMEOUT
		timeout = s + 5;
		//gpib_info();
		while ((PIND & _BV(G_NDAC)) && (s <= timeout)) {
			if (s == timeout) {
				uart_puts("\n\rError: NDAC timeout\n\r");
				return 0xff;
			}
		}
#else
		loop_until_bit_is_clear(PIND,G_NDAC);
#endif

		DDRA = 0x00;
		if (c & 0x01) {
			assign_bit(DDRA, PORTA, PA0);
		} else {
			release_bit(DDRA, PORTA, PA0)
		}

		if (c & 0x02) {
			assign_bit(DDRA, PORTA, PA1)
		} else {
			release_bit(DDRA, PORTA, PA1);
		}

		if (c & 0x04) {
			assign_bit(DDRA, PORTA, PA2);
		} else {
			release_bit(DDRA, PORTA, PA2);
		}

		if (c & 0x08) {
			assign_bit(DDRA, PORTA, PA3);
		} else {
			release_bit(DDRA, PORTA, PA3);
		}

		if (c & 0x10) {
			assign_bit(DDRA, PORTA, PA4);
		} else {
			release_bit(DDRA, PORTA, PA4);
		}

		if (c & 0x20) {
			assign_bit(DDRA, PORTA, PA5);
		} else {
			release_bit(DDRA, PORTA, PA5);
		}

		if (c & 0x40) {
			assign_bit(DDRA, PORTA, PA6);
		} else {
			release_bit(DDRA, PORTA, PA6);
		}

		if (c & 0x80) {
			assign_bit(DDRA, PORTA, PA7);
		} else {
			release_bit(DDRA, PORTA, PA7);
		}

		// wait until listeners release NRFD
		//uart_puts("1");
		release_bit(DDRD, PORTD, G_NRFD);
#ifdef WITH_TIMEOUT
		//gpib_info();
		timeout = s + 5;
		while (!(PIND & _BV(G_NRFD)) && (s <= timeout)) {
			if (s == timeout) {
				uart_puts("\n\rError: NRFD timeout\n\r");
				return 0xff;
			}
		}
#else
		loop_until_bit_is_set(PIND,G_NRFD);
#endif

		// assign EOI during transmission of only last byte
		if ((i == length - 1) && !attention) {
			//uart_puts("\n\rE\n\r");
			assign_bit(DDRD, PORTD, G_EOI);
		}

		// assign DAV, data valid for listeners
		assign_bit(DDRD, PORTD, G_DAV);

		// wait for NDAC release
		//uart_puts("2");
		release_bit(DDRD, PORTD, G_NDAC);
		loop_until_bit_is_set(PIND, G_NDAC);

		// release DAV, data not valid anymore
		release_bit(DDRD, PORTD, G_DAV);

		// reset Port to all input
		DDRA = 0x00;

		//uart_puts("3\r\n");
	}

	if (attention) {
		// assign ATN for commands
		release_bit(DDRD, PORTD, G_ATN);
	}

	// clear talk state.Controller does not talk anymore.
	controller.talks = 0;

	return 0x00;
}

/**
 * print some useful info about bus state (for example value of handshake pins)
 */
void gpib_info(void) {
	uchar dav, nrfd, ndac, eoi, atn, srq, ifc, ren;
	extern uchar buf[80];

	sprintf(buf, "Partner address: primary: %u, secondary: %u\n\r",
			gpib_get_partner_pad(), gpib_get_partner_sad());
	uart_puts(buf);

	uart_puts("Partner list\n\r");
	for (int i = 0; i < MAX_PARTNER; i++) {
		if (controller.partners[i].primary != ADDRESS_NOT_SET) {
			sprintf(buf, "Partner address: primary: %u, secondary: %u\n\r",
					controller.partners[i].primary,
					controller.partners[i].secondary);
			uart_puts(buf);
		}
	}

	dav = bit_is_set(PIND, G_DAV);
	nrfd = bit_is_set(PIND, G_NRFD);
	ndac = bit_is_set(PIND, G_NDAC);
	eoi = bit_is_set(PIND, G_EOI);
	atn = bit_is_set(PIND, G_ATN);
	srq = bit_is_set(PIND, G_SRQ);
	ifc = bit_is_set(PINB, G_IFC);
	ren = bit_is_set(PINB, G_REN);
	//d = PINA;
	//di = d ^ 0xff;
	if (dav == 0x00)
		dav = '0';
	else
		dav = '1';
	if (nrfd == 0x00)
		nrfd = '0';
	else
		nrfd = '1';
	if (ndac == 0x00)
		ndac = '0';
	else
		ndac = '1';
	if (eoi == 0x00)
		eoi = '0';
	else
		eoi = '1';
	if (atn == 0x00)
		atn = '0';
	else
		atn = '1';
	if (srq == 0x00)
		srq = '0';
	else
		srq = '1';
	if (ifc == 0x00)
		ifc = '0';
	else
		ifc = '1';
	if (ren == 0x00)
		ren = '0';
	else
		ren = '1';

	sprintf(buf,
			"dav=%c,nrfd=%c,ndac=%c, eoi=%c, ifc=%c,ren=%c,atn=%c,srq=%c\n\r",
			dav, nrfd, ndac, eoi, ifc, ren, atn, srq);
	uart_puts(buf);
}

/**
 * Execute serial polling
 *
 * We return the physical address of the device that created the SRQ.
 *
 * Note that this code is not tested since moving to primary/secondary addresses.
 * I haven't looked how a device with two byte address behaves.
 */
uchar gpib_serial_poll(void) {
	uchar b, e;
	uchar primary = 0, secondary = 0, found = 0, foundPhysical = 0;
	int i;

	// send UNT and UNL commands (unlisten and untalk)
	// effect: all talker stop talking and all listeners stop listening
	cmd_buf[0] = G_CMD_UNT;
	gpib_cmd(cmd_buf, 1);
	cmd_buf[0] = G_CMD_UNL;
	gpib_cmd(cmd_buf, 1);

	// serial poll enable
	// effect: all devices will send status byte instead of normal data when addressed
	// as talker
	//uart_puts("before SPE\r\n");
	cmd_buf[0] = G_CMD_SPE;
	gpib_cmd(cmd_buf, 1);
	//uart_puts("after SPE\r\n");

	// searching for SRQ emitter in a loop ...
	for (i = 0; (controller.partners[i].primary != 0x00) && !found; i++) {

		// set partner to talker mode
		primary = address2TalkerAddress(controller.partners[i].primary);
		secondary = secondaryAdressToAdressByte(
				controller.partners[i].secondary);

		cmd_buf[0] = primary;
		//uart_puts("before talker address write\r\n");
		gpib_cmd(cmd_buf, 1);
		//uart_puts("after talker address write\r\n");
		// handle secondary address if required
		if (secondary != ADDRESS_NOT_SET) {
			cmd_buf[0] = secondary;
			//uart_puts("before talker address write\r\n");
			gpib_cmd(cmd_buf, 1);
		}

		// now receive data
		//uart_puts("before status byte receive\r\n");
		e = gpib_receive(&b);
		//uart_puts("after status byte receive\r\n");
		// status byte is now in b
		sprintf((char*) cmd_buf,
				"Status byte from device 0x%02x (physical address) = 0x%02x\n\r",
				TalkerAddress2Address(primary), b);
		uart_puts((char*) cmd_buf);

		// send UNT and UNL commands (unlisten and untalk)
		// effect: all talker stop talking and all listeners stop listening
		cmd_buf[0] = G_CMD_UNT;
		gpib_cmd(cmd_buf, 1);
		cmd_buf[0] = G_CMD_UNL;
		gpib_cmd(cmd_buf, 1);

		if (b & (1 << 6)) {
			found = primary;
			foundPhysical = TalkerAddress2Address(found);
			// bit 6 of status byte of SRQ emitter is 1
			// when reading status byte from emitter, he releases SRQ line (may also be tested here)
			sprintf((char*) cmd_buf,
					"SRQ emitter is device = 0x%02x (physical address), secondary = 0x%02x\n\r",
					foundPhysical, secondary);
			uart_puts((char*) cmd_buf);
		}
	}

	// serial poll disable
	// effect: all devices will return to normal behaviour as talker
	cmd_buf[0] = G_CMD_SPD;
	//uart_puts("before SPD\r\n");
	gpib_cmd(cmd_buf, 1);
	//uart_puts("after SPD\r\n");

	// return SRQ emitter address if found
	return foundPhysical;
}

/**
 * Set device to be controlled.
 * \param address Address of device.
 */
void gpib_set_partner_address(uchar primary, uchar secondary) {
	controller.partner.primary = primary;
	controller.partner.secondary = secondary;
}

/**
 * Set device to be controlled.
 * \param address Address of device.
 */
void gpib_set_partner_secondary(uchar secondary) {
	controller.partner.secondary = secondary;
}

/**
 * Get primary address of device currently controlled.
 * \returns primary address Address of device.
 */
uchar gpib_get_partner_pad(void) {
	return controller.partner.primary;
}

/**
 * Get secondary address of device currently controlled.
 * \returns secondary address Address of device.
 */
uchar gpib_get_partner_sad(void) {
	return controller.partner.secondary;
}

/**
 * Get controller address.
 * \returns address of controller.
 */
uchar gpib_get_address(void) {
	return controller.myaddress;
}

void gpib_set_flavour(uchar flavour) {
	controller.flavour = flavour;
}

uchar gpib_get_flavour(void) {
	return controller.flavour;
}

/**
 * Clear partners list
 */
void gpib_clear_partners() {
	for (int i = 0; i < MAX_PARTNER; i++) {
		controller.partners[i].primary = ADDRESS_NOT_SET;
	}
}

/**
 * Add partner to list of known devices. Only these acre scanned during a serial poll.
 */
uchar gpib_add_partner_address(uchar primary, uchar secondary) {
	int i;
	for (i = 0;
			i < MAX_PARTNER && controller.partners[i].primary != ADDRESS_NOT_SET;
			i++) {
	}
	if (i == MAX_PARTNER) {
		uart_puts("Too much partners.\n\r");
		return 1;
	}
	controller.partners[i].primary = primary;
	controller.partners[i].secondary = secondary;
	return 0;
}

/**
 * Remove partner from list of known devices.
 */
uchar gpib_remove_partner_address(uchar primary, uchar secondary) {
	int i;
	for (i = 0;
			i < MAX_PARTNER
					&& (controller.partners[i].primary != primary
							|| controller.partners[i].secondary != secondary);
			i++) {
	}
	if (i == MAX_PARTNER) {
		uart_puts("Partner unknown.\n\r");
		return 1;
	}
	controller.partners[i].primary = ADDRESS_NOT_SET;
	controller.partners[i].secondary = ADDRESS_NOT_SET;
	return 0;
}

