#
# Generates HEX file for upload to microcontroller.
# Requires avr-gcc toolchain.
#

CPUFLAGS = -mmcu=atmega32
CFLAGS += -Wl,-Map,gpib.map -u,vprintf $(CPUFLAGS)
OBJS =  main_gpib.o gpib.o timer16.o uart.o
LIBS =

.c.o:
	avr-gcc $(CFLAGS) -u,vprintf -o $?

controller: gpib.elf gpib.lss gpib.hex sizedummy

# compile
gpib.elf: $(OBJS)
	# create ELF file
	avr-gcc $(CFLAGS) -o "gpib.elf" $(OBJS) $(LIBS)

gpib.lss: gpib.elf
	# dump objects
	avr-objdump -h -S gpib.elf  >"gpib.lss"

gpib.hex: gpib.elf
	# Create Flash image (ihex format)
	avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex gpib.elf  "gpib.hex"

sizedummy: gpib.elf
	# print size
	avr-size --format=avr $(CPUFLAGS) gpib.elf