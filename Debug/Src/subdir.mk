################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/006spi_cmd_handling.c 

OBJS += \
./Src/006spi_cmd_handling.o 

C_DEPS += \
./Src/006spi_cmd_handling.d 


# Each subdirectory must supply rules for building sources it contributes
Src/006spi_cmd_handling.o: ../Src/006spi_cmd_handling.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I"/Users/junaidkhan/Documents/MCU-Udemy/stm32f4xx-drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/006spi_cmd_handling.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

