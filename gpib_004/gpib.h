/*************************************************************************
 Author:   	$Author: dennis $
 File:     	$HeadURL: file://localhost/home/dennis/svn-store/avr-source/gpib_004/gpib.h $
 Date:  		$Date: 2012-04-14 17:41:46 +0200 (Sa, 14 Apr 2012) $
 Revision: 	$Revision: 688 $
 Id: 		$Id: gpib.h 688 2012-04-14 15:41:46Z dennis $
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
 *
 *  @author Dennis Dingeldein  http://www.dingeldein-online.de
 */

#ifndef GPIB_H_
#define GPIB_H

#include "defs.h"

/** controller "flavours", controls special devices/companies */
#define FLAVOUR_NONE 0
#define FLAVOUR_TEK 1

/** undefined address */
#define ADDRESS_NOT_SET 0xff

/** Handshake lines: DAV, NRFD and NDAC on Port D */
#define G_DAV  PD2
#define G_NRFD PD3
#define G_NDAC PD5

/** management lines: EOI, SRQ and ATN on Port D, IFC and REN on Port B */
#define G_EOI  PD4
#define G_SRQ  PD6
#define G_ATN  PD7
#define G_IFC  PB0
#define G_REN  PB1

/** GPIB command codes */
#define G_CMD_UNL 0x3f
#define G_CMD_UNT 0x5f
#define G_CMD_SPE 0x18
#define G_CMD_SPD 0x19
#define G_CMD_DCL 0x14

/** important ASCII codes */
#define ASCII_CODE_CR 0x0d
#define ASCII_CODE_LF 0x0a

/**
 * GPIB Address scheme: physical addresses are 0,1,2,3,...
 * Listener addresses are physical address + 0x20
 * Talker address is physical address + 0x40
 */
/** calculate listener address from physical device address */
#define address2ListenerAddress(adr) (adr+0x20)
/** calculate talker address from physical device address */
#define address2TalkerAddress(adr) (adr+0x40)

/** calculate physical address from listener address */
#define listenerAddress2Address(adr) (adr-0x20)
/** calculate physical address from talker address */
#define TalkerAddress2Address(adr) (adr-0x40)

/** adress byte for secondary address is (adr | 0110.0000=0x60) */
#define secondaryAdressToAdressByte(adr) (adr|0x60)
/** secondary address is adress byte (adrbyte & 1001.1111=0x9f) */
#define secondaryAdressByteToAdress(adr) (adr&0x9f)

/** maximum size of active partner list */
#define MAX_PARTNER 5 

// management functions for controller
extern void gpib_init(void);
extern void gpib_untalkUnlisten();
extern void gpib_write_epilogue(uchar attention);
extern void gpib_write_prologue(uchar attention) ;

extern void gpib_set_flavour(uchar flavour);
extern uchar gpib_get_flavour(void);

extern void gpib_controller_assign(uchar address);
extern void gpib_controller_release(void);
extern uchar gpib_cmd(uchar *bytes, int length);
uchar gpib_serial_poll(uint8_t *primary_v, uint8_t* secondary_v);

extern void gpib_set_partner_address(uchar primary, uchar secondary);
extern void gpib_set_partner_secondary(uchar address);
extern uchar gpib_get_partner_pad(void);
extern uchar gpib_get_partner_sad(void);
extern uchar gpib_get_address(void);

extern uchar gpib_add_partner_address(uchar primary, uchar secondary);
extern uchar gpib_remove_partner_address(uchar primary, uchar secondary);
extern void gpib_clear_partners();

// listener functions
extern void gpib_prepare_read() ;
extern uchar gpib_receive(uchar *byte);

// talker functions
extern void gpib_prepare_write() ;
extern uchar gpib_write_byte(uchar c, uchar isLastByte);
extern void gpib_write_string(uchar *s);
extern void gpib_write_command(uchar *s);

// find all devices up to address maxAdress
void gpib_find_devices(uchar maxAddress) ;

// just for code testing
extern void gpib_info(void);

#endif /*GPIB_H_*/
