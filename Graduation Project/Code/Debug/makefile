################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include MCAL/subdir.mk
-include HAL/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Autonomous_Parking.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Autonomous_Parking.list \

OBJCOPY_BIN += \
Autonomous_Parking.bin \


# All Target
all: Autonomous_Parking.elf secondary-outputs

# Tool invocations
Autonomous_Parking.elf: $(OBJS) $(USER_OBJS) F:\studying\final_project\self_parking\New\ folder\ (3)\Autonomous_Parking\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Autonomous_Parking.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"F:\studying\final_project\self_parking\New folder (3)\Autonomous_Parking\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Autonomous_Parking.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Autonomous_Parking.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Autonomous_Parking.list"
	@echo 'Finished building: $@'
	@echo ' '

Autonomous_Parking.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Autonomous_Parking.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
