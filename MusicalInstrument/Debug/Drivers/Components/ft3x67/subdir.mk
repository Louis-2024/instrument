################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/ft3x67/ft3x67.c 

OBJS += \
./Drivers/Components/ft3x67/ft3x67.o 

C_DEPS += \
./Drivers/Components/ft3x67/ft3x67.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/ft3x67/%.o Drivers/Components/ft3x67/%.su: ../Drivers/Components/ft3x67/%.c Drivers/Components/ft3x67/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Inc -I"/Users/weihengxiao/Desktop/MusicalInstrument/Inc" -I"/Users/weihengxiao/Desktop/MusicalInstrument/Drivers/Components/hts221" -I"/Users/weihengxiao/Desktop/MusicalInstrument/Drivers/Components" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-ft3x67

clean-Drivers-2f-Components-2f-ft3x67:
	-$(RM) ./Drivers/Components/ft3x67/ft3x67.d ./Drivers/Components/ft3x67/ft3x67.o ./Drivers/Components/ft3x67/ft3x67.su

.PHONY: clean-Drivers-2f-Components-2f-ft3x67

