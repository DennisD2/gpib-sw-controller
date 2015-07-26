/*************************************************************************
Author:   	$Author: dennis $
File:     	$HeadURL: file://localhost/home/dennis/svn-store/avr-source/gpib_004/defs.h $
Date:  		$Date: 2012-03-25 18:14:48 +0200 (So, 25 Mrz 2012) $ 
Revision: 	$Revision: 685 $ 
Id: 		$Id: defs.h 685 2012-03-25 16:14:48Z dennis $ 
Licence:	GNU General Public License

DESCRIPTION:
          standard definitions used in all .h and .c files of project
 *************************************************************************/
#ifndef DEFS_H_
#define DEFS_H_

/** software revision number */
#define REVISION "0.8"

typedef unsigned char uchar;
typedef unsigned int uint;
typedef unsigned long ulong;

#ifndef F_CPU
#error "You must define F_CPU value (CPU frequency in Hz)"
/*#define F_CPU 1000000*/
#endif

#define UART_BAUD_RATE      38400     /**< baudrate, e.g 19200 */
#define DI() uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) )

#endif /*DEFS_H_*/
