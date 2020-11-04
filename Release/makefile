################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
nezutaka_git.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
nezutaka_git.list \

OBJCOPY_BIN += \
nezutaka_git.bin \


# All Target
all: nezutaka_git.elf secondary-outputs

# Tool invocations
nezutaka_git.elf: $(OBJS) $(USER_OBJS) C:\Users\junmo\OneDrive\ -\ kcg.ac.jp\BACKUP\mause\nezutaka\nezutaka_git\STM32F405RGTX_FLASH.ld
	arm-none-eabi-gcc -o "nezutaka_git.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\junmo\OneDrive - kcg.ac.jp\BACKUP\mause\nezutaka\nezutaka_git\STM32F405RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="nezutaka_git.map" -Wl,--gc-sections -static -L"C:\Users\junmo\OneDrive - kcg.ac.jp\BACKUP\mause\nezutaka\nezutaka_git" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

nezutaka_git.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "nezutaka_git.list"
	@echo 'Finished building: $@'
	@echo ' '

nezutaka_git.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "nezutaka_git.bin"
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
