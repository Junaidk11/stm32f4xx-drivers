/*
 * 004spi_driver_test.c
 *
 *  Created on: Jul 6, 2020
 *      Author: junaidkhan
 */

#include "stm32f40xx.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include "string.h"

/**
 *  Pins for Configuring SPI2 for transmission
 * 
 * PB12 -> SPI2_NSS
 * PB13 ->  SPI2_SCLK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSO
 * 
 * Alternate Functionality mode = 5; Needed for configuring the I/O pins for alternate functionality 
 * 
 **/

void SPI2_GPIOInits(){

	// Enable clock access to GPIOB Port before making any configurations. 
	GPIOB_PERIPH_CLOCK_EN(); 

	// Create a GPIO handle, which will hold the base address and Pin configurations 
	GPIO_Handle_t SPIPins; 
	SPIPins.pGPIOx_BASEADDR = GPIOB; 

	// Set Pin Configurations for each I/O Pin
	SPIPins.PinConfig.PinMode = GPIO_PIN_ALTFUNC_MODE; 
	SPIPins.PinConfig.PinAltFunMode = 5; 
	SPIPins.PinConfig.PinOType = GPIO_PUSH_PULL_OUTPUT_CONFIG; // Output Drain Type is required for I2C -> specification insists.
	SPIPins.PinConfig.PinPuPdControl = GPIO_PIN_NO_PUPD;  // Don't need PU/PD for Push-pull output configuration
	SPIPins.PinConfig.PinSpeed = GPIO_PIN_HIGH_SPEED;  // Doesn't matter, can set to any speed

	// Set Pin number and call the GPIOInit to initialize each of the PBx pins.

	// SPI2_NSS
	SPIPins.PinConfig.PinNumber = GPIO_PIN_12; 
	GPIO_Init(&SPIPins); 

	// SPI2_SCLK 
	SPIPins.PinConfig.PinNumber = GPIO_PIN_13; 
	GPIO_Init(&SPIPins); 

	// SPI2_MISO
	SPIPins.PinConfig.PinNumber = GPIO_PIN_14; 
	GPIO_Init(&SPIPins);
	
	// SPI2_MOSI
	SPIPins.PinConfig.PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins); 


}

void SPI2_Inits(){


		// Enable clock access to the peripheral before making any configurations. 
		SPI2_PERIPH_CLOCK_EN(); 

		// Create SPI Handle 
		SPI_Handle_t SPI2Handle; 

		// Set BaseAddress to SPI2 
		SPI2Handle.pSPIx_BASEADDR = SPI2;

		// Configure the SPI2 Peripheral as desired. 
		SPI2Handle.SPIConfig.DeviceMode = SPI_IN_MASTER_MODE;  
		SPI2Handle.SPIConfig.BusConfig = SPI_IN_FULL_DUPLEX_MODE; 
		SPI2Handle.SPIConfig.DataFrameFormat = SPI_DATAFRAME_8BITS; 
		SPI2Handle.SPIConfig.ClockPolarity = SPI_SCLK_LOW; 
		SPI2Handle.SPIConfig.ClockPhase = SPI_DATA_SAMPLED_ON_LEADING_EDGE; 
		SPI2Handle.SPIConfig.SclkSpeed = SPI_PERIPHERAL_CLOCK_DIV8; // Serial Clock is at 2 MHz, which means Prescaler = 8, System Clock source is Internal RC oscillator, producing 16MHz
		SPI2Handle.SPIConfig.SlaveManagementType = SPI_SOFTWARE_SLAVE_MANAGEMENT_DI; // 1 slave, so we can disable software slave management, i.e. enable hardware slave management

		// Call the Init API to initialize the configured SPI2 peripheral.
		SPI_Init(&SPI2Handle);

}

void GPIO_ButtonInits(){
    	/*   The following statements are for configuring the user button PA0 as an input pin. */

	GPIO_Handle_t gpio_user_button; 				// Instantiate a GPIO handle variable to hold pin configuration information and gain access to the API functions.
	gpio_user_button.pGPIOx_BASEADDR = GPIOA; 	// Set the base-address of the Port you're configuring - User button is connected to Port A, Pin 0.

	// Now, use the Pin structure of the GPIO handle to configure the I/O pin 0 in input mode.
	gpio_user_button.PinConfig.PinNumber = GPIO_PIN_0;
	gpio_user_button.PinConfig.PinMode = GPIO_PIN_INPUT_MODE; 		 // Set the pin in Output mode.
	//gpio_push_pull.PinConfig.PinOType = GPIO_PUSH_PULL_OUTPUT_CONFIG; //PA0 is being configured as an input pin, this is irrelevant to it.
	gpio_user_button.PinConfig.PinPuPdControl = GPIO_PIN_NO_PUPD;      // Stm32f407-discovery schematic shows an external pull-down resistor, so don't need to enable internal pull-down resistor.
	gpio_user_button.PinConfig.PinSpeed = GPIO_PIN_MEDIUM_SPEED;   // Speed doesn't really matter

	// Now, enable clock access to GPIO Port D, use the RCC clock enable MACROS defined in the device specific header file or:
	// Use the clock control API defined in GPIO driver - the better option.

	GPIO_ClockControl(GPIOA, ENABLE);

	// Now, call the Init API to configure the physical address of Port D
	GPIO_Init(&gpio_user_button);

}


void Delay()
{
    int i = 0;
    for (; i < 500000 / 2; i++);
}
int main(){

    // 1. Call Function to configure the I/O pin PA0 as button 
    GPIO_ButtonInits();

	// 2. Call Function to configure the I/O pins above to be used as SPI2 pin. 
	SPI2_GPIOInits();

	// 3. Call Function to Configure the SPI2 Peripheral 
	SPI2_Inits();

	// 4. Set SSOE pin to enable Slave Select Output, which will tie Peripheral Enable bit to the NSS output 
    // i.e. when SPI enabled, NSS will be driven low to initiate communication with the slave --> Only if the SPI is in master mode, which is the case here.
    SPI_SSOEConfig(SPI2, ENABLE);
	

    while(1)
    {
            while(!(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0))); // 5. Wait here until PA0 is pressed. 
            
            Delay();    // Cause a delay to avoid button de-bouncing 

            // 6. Enable the SPI Peripheral
            SPI_PeripheralControl(SPI2, ENABLE); 
            
            // 7. Data to send
            char data[] ="HELLO WORLD!"; 
                
            // 8. Send data length information to the slave before sending data,
            uint8_t dataLength = strlen(data);
            SPI_SendData(SPI2, &dataLength, 1);  // 1 byte of data that holds the length of the actual data to be sent 


            // 9. Call SPI_SendData Function to send actual data 
            SPI_SendData(SPI2,(uint8_t *)data,strlen(data)); 

            // 10. Test SPI Busy Flag before closing the SPI peripheral 
            // SPI_SR_BSY flag is controlled by hardware and it is cleared when SPI is not busy
            while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY)); // SPI busy, you stay here

            // 11. Disable SPI Peripheral 
            SPI_PeripheralControl(SPI2, DISABLE);
    }

	
}
