################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/FinalProject_MPU.c \
../src/FusionAhrs.c \
../src/FusionBias.c \
../src/FusionCompass.c \
../src/cr_startup_lpc407x_8x.c \
../src/crp.c \
../src/sysinit.c 

OBJS += \
./src/FinalProject_MPU.o \
./src/FusionAhrs.o \
./src/FusionBias.o \
./src/FusionCompass.o \
./src/cr_startup_lpc407x_8x.o \
./src/crp.o \
./src/sysinit.o 

C_DEPS += \
./src/FinalProject_MPU.d \
./src/FusionAhrs.d \
./src/FusionBias.d \
./src/FusionCompass.d \
./src/cr_startup_lpc407x_8x.d \
./src/crp.d \
./src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M4 -D__USE_LPCOPEN -D__LPC407X_8X__ -D__REDLIB__ -I"/Users/dali/Documents/LPCXpresso_8.2.2/workspace/FinalProject_MPU/inc" -I"/Users/dali/Documents/LPCXpresso_8.2.2/workspace/lpc_board_ea_devkit_4088/inc" -I"/Users/dali/Documents/LPCXpresso_8.2.2/workspace/lpc_chip_40xx/inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fsingle-precision-constant -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


