################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/gpio_driver.c \
../drivers/src/i2c_driver.c \
../drivers/src/spi_driver.c 

OBJS += \
./drivers/src/gpio_driver.o \
./drivers/src/i2c_driver.o \
./drivers/src/spi_driver.o 

C_DEPS += \
./drivers/src/gpio_driver.d \
./drivers/src/i2c_driver.d \
./drivers/src/spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/gpio_driver.o: ../drivers/src/gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I"/Users/junaidkhan/Documents/MCU-Udemy/stm32f4xx-drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/src/i2c_driver.o: ../drivers/src/i2c_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I"/Users/junaidkhan/Documents/MCU-Udemy/stm32f4xx-drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/i2c_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/src/spi_driver.o: ../drivers/src/spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I"/Users/junaidkhan/Documents/MCU-Udemy/stm32f4xx-drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/spi_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

