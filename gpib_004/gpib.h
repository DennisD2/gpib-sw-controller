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

/** default address to be used as partner device */
#define DEFAULT_PARTNER_ADDRESS 0x01

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

/** maximum size of active partner list */
#define MAX_PARTNER 5 

// management functions for controller
extern void gpib_init( void );
extern void gpib_controller_assign( uchar address );
extern void gpib_controller_release( void );
extern uchar gpib_cmd( uchar *bytes, int length );
uchar gpib_serial_poll( void );
extern void gpib_set_partner( uchar address );
extern uchar gpib_get_partner( void );
extern uchar gpib_get_address( void );

// listener functions
extern uchar gpib_receive( uchar *byte );

// talker functions
extern uchar gpib_write( uchar *bytes, int length );

// just for code testing
extern void gpib_info( void );

#endif /*GPIB_H_*/
