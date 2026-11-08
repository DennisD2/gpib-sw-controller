# GPIB software only controller code
GPIB software only controller code for AVR microcontrollers.

## Introduction
These files implement a GPIB controller. No special hardware is used, the port
lines of the AVR is used 'as is'.

### Hardware
To run the software, it must be compiled according to your hardware.
See gpib.h for the assignments of AVR port lines to GPIB interface lines.
You can change the defines in gpib.h according to your hardware.
I used for testing the AVR board from Pollin priced ca. 15 Euro. Every other board
is possible as long as you have enough port lines available. 16 ports are required.

### GPIB bus setup
Set up GPIB bus addresses for your devices. The setup is done in/at the device. Check device
manual. The controller can handle primary only and primary/secondary addresses.

### Order of switching on devices on bus
This is a little bit tricky. Best order is to switch on controller, and after that switch on the
devices.

The software keeps a list of "known devices". These are devices that are included in a
serial poll (after a SRQ). You must add devices manually to the list. If a device not in the
list generates a SRQ this SRQ is not correctly handled.

### Software
This code implements a kind of GPIB controller. It understands a few built in commands.
The hardware should be connected via GPIB interface, the controlling computer/terminal via RS232.
Then the following commands can be used. <n> is a GPIB device address, e.g. 3.:
* .a n - set partner device address to value n.
The partner device is the device to control.
* .+ n - add partner device address to list of known devices. Used for SRQ handling.
* .- n - remove partner device address from list of known devices. Used for SRQ handling.
* .h - print some help.
* .i - print out status of some GPIB lines and current partner address.

When in normal input mode, device commands can be entered. For commands that will have an answer
from the device under control, the answer is print out to the terminal. The controller is also
able to do a serial poll.
The code was tested with the following devices:
* Tektronix 2432a oscilloscope
* Tektronix 1241 logic analyzer
* HP3478 multimeter
* HP75000 VXI mainframe with several modules installed (uses secondary addresses)

The implementation may not work well with your device.But it's worth a try :-)

### Files
The implementation of GPIB protocol and controller functions itself is in [gpib.h](gpib.h) 
and [gpib.c](gpib.c).
The controller is implemented in [main_gpib.c](main_gpib.c) as a big never-ending loop.

Errors are usually timeouts because the device behaves different
then the controller expected. The timeout functionality is implemented as 16 bit timer in
[timer16.h](timer16.h) and [timer16.c](timer16.c). For generic common defines the file [defs.h](defs.h) is used.
The [Doxyfile](Doxyfile) file contains doxygen controls (automatic creation of documentation from source).

### Code Description
See [doc/html/index.html](doc/html/index.html)

### Code status
This is old, nearly ancient code with latest code changes from 2012.
I keep the code here to have a reference when diving into GPIB.

My self-made hardware was not needed at some point in time and is gone. 
So only software is left. 

Nowadays, I am using a USB-to-GPIB adapter from National. At least
the originals fron NI work well with Linux. Alas, it is required to add linuxgpib.

### Some arbitrary GPIB commands
Here are some commands for testing.
#### Tek 2432a
```bash
id?
INIT
MEASUREMENT WINDOW:ON
CURSOR FUNCTION: TIME,TARGET:CH1,UNITS:TIME:BASE
CURSOR TPOS:ONE:200,TPOS:TWO:500
message 5:"a test message"
RUN?
CH1?
wavfrm?
curve?
wfmpre?
data encdg:ascii
data encdg:ascii
```

#### Tek 1241
```bash
id?
init
dt?
acqmem?
refmem?
```

#### HP3768
```bash
H1?
```

#### HP75000 / E1300 board using secondary address feature
```bash
*rst
*idn?
SYST:TIME?
```

### Licence
This code can be used according to GNU General Public License.

