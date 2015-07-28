################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
gpib_004.lss \

FLASH_IMAGE += \
gpib_004.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: gpib_004.elf secondary-outputs

# Tool invocations
gpib_004.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,gpib_004.map -u,vprintf -mmcu=atmega32 -u,vprintf -o "gpib_004.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

gpib_004.lss: gpib_004.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S gpib_004.elf  >"gpib_004.lss"
	@echo 'Finished building: $@'
	@echo ' '

gpib_004.hex: gpib_004.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex gpib_004.elf  "gpib_004.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: gpib_004.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega32 gpib_004.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) gpib_004.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
