################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Projekt_SM.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Projekt_SM.list \


# All Target
all: Projekt_SM.elf secondary-outputs

# Tool invocations
Projekt_SM.elf: $(OBJS) $(USER_OBJS) C:\Users\Radek\STM32CubeIDE\workspace_1.0.2\Projekt_SM\STM32F746ZGTX_FLASH.ld
	arm-none-eabi-gcc -o "Projekt_SM.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\Radek\STM32CubeIDE\workspace_1.0.2\Projekt_SM\STM32F746ZGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Projekt_SM.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Projekt_SM.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Projekt_SM.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
