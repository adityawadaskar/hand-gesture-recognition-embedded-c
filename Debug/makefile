################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: FinalProject_MPU.axf

# Tool invocations
FinalProject_MPU.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/Users/dali/Documents/LPCXpresso_8.2.2/workspace/lpc_board_ea_devkit_4088/Debug" -L"/Users/dali/Documents/LPCXpresso_8.2.2/workspace/lpc_chip_40xx/Debug" -Xlinker -Map="FinalProject_MPU.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -T "FinalProject_MPU_Debug.ld" -o "FinalProject_MPU.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) FinalProject_MPU.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "FinalProject_MPU.axf"; # arm-none-eabi-objcopy -v -O binary "FinalProject_MPU.axf" "FinalProject_MPU.bin" ; # checksum -p LPC4088 -d "FinalProject_MPU.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
