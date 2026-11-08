
OBJS =  main_gpib.o gpib.o timer16.o uart.o
LIBS =

.c.o:
	avr-gcc -mmcu=atmega32 -u,vprintf -o $?

controller: gpib.elf gpib.lss gpib.hex sizedummy

# compile
gpib.elf: $(OBJS)
	avr-gcc -Wl,-Map,gpib.map -u,vprintf -mmcu=atmega32 -u,vprintf -o "gpib.elf" $(OBJS) $(LIBS)

gpib.lss: gpib.elf
	avr-objdump -h -S gpib.elf  >"gpib_004.lss"

gpib.hex: gpib.elf
	# Create Flash image (ihex format)
	avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex gpib.elf  "gpib.hex"

sizedummy: gpib.elf
	# print size
	avr-size --format=avr --mcu=atmega32 gpib_004.elf