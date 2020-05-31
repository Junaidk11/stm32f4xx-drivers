################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/gpio_driver.c 

OBJS += \
./drivers/src/gpio_driver.o 

C_DEPS += \
./drivers/src/gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/gpio_driver.o: ../drivers/src/gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"/Users/junaidkhan/Desktop/MCU-Udemy/stm32f4xx-drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

