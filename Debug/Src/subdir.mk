################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/004spi_driver_test.c 

OBJS += \
./Src/004spi_driver_test.o 

C_DEPS += \
./Src/004spi_driver_test.d 


# Each subdirectory must supply rules for building sources it contributes
Src/004spi_driver_test.o: ../Src/004spi_driver_test.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"/Users/junaidkhan/Desktop/MCU-Udemy/stm32f4xx-drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/004spi_driver_test.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

