#
# Generates HEX file for upload to microcontroller.
# Requires avr-gcc toolchain.
# OpenSuse toolchain looks bad, so I needed to call CC1 and AS manually.
#
CROSS_PRE=-B /usr/bin/avr-
CC=avr-gcc
CC1=/opt/cross/libexec/gcc/avr/5.4.0/cc1
AS=avr-as

CPUFLAGS = -mmcu=atmega32
FREQ=16000000
CPU=atmega32
#CFLAGS += -S -v -Wl,-Map,gpib.map -u,vprintf -mmcu=$(CPU) -I/usr/avr/sys-root/include $(CROSS_PRE)
CFLAGS1 = -S -Wall -g -O0 -mmcu=$(CPU) -DF_CPU=$(FREQ)  -I/usr/avr/sys-root/include $(CROSS_PRE)
OBJS =  main_gpib.o gpib.o timer16.o uart.o
LIBS =

.c.s:
	avr-gcc $(CFLAGS1) -o"$@" "$<"

.c.o:
	# dot c to dot o
	$(CC1) -quiet -I /usr/avr/sys-root/include -imultilib avr5 \
		-D__AVR_ATmega32__ -D__AVR_DEVICE_NAME__=atmega32 -DF_CPU=$(FREQ) "$<" -mn-flash=1 \
		-mno-skip-bug -quiet -dumpbase main_gpib.c -mmcu=avr5 -auxbase-strip -g -O0 -Wall -version -o "$@".s
	/usr/bin/avr-as -v -mmcu=$(CPU) -mno-skip-bug -o"$@" "$@".s

controller: gpib.elf gpib.lss gpib.hex sizedummy

gpib.o: gpib.s

# compile
gpib.elf: $(OBJS)
	# create ELF file
	#avr-gcc $(CFLAGS1) -o "gpib.elf" $(OBJS) $(LIBS)
	avr-gcc -Wl,-Map,gpib.map -g -O0 -u,vprintf -mmcu=atmega32 -DF_CPU=16000000 -B /usr/bin/avr- -B/usr/avr/sys-root/lib/avr5 -L /usr/avr/sys-root/lib/avr5 \
 		-o "gpib.elf" main_gpib.o gpib.o timer16.o uart.o

gpib.lss: gpib.elf
	# dump objects
	avr-objdump -h -S gpib.elf  >"gpib.lss"

gpib.hex: gpib.elf
	# Create Flash image (ihex format)
	avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex gpib.elf  "gpib.hex"

sizedummy: gpib.elf
	# print size
	avr-size --format=avr --mcu=atmega32 gpib.elf

clean:
	rm -f *.o *.eld *.lss