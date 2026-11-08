/*************************************************************************
Author:   	$Author: dennis $
File:     	$HeadURL: file://localhost/home/dennis/svn-store/avr-source/gpib_004/timer16.h $
Date:  		$Date: 2008-05-11 07:57:02 +0200 (So, 11 Mai 2008) $ 
Revision: 	$Revision: 65 $ 
Id: 		$Id: timer16.h 65 2008-05-11 05:57:02Z dennis $ 
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

#ifndef TIMER16_H_
#define TIMER16_H_

#include "defs.h"

#define DEBOUNCE 256L /**< number of calls per second for timer ISR */

extern volatile uchar s; /**< second value, increased by one every second */

void timer16_init( void );

#endif /*TIMER16_H_*/
