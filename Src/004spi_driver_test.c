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
		SPI2Handle.SPIConfig.SclkSpeed = SPI_PERIPHERAL_CLOCK_DIV2; // Serial Clock is at lowest possible at 8 MHz as Prescaler = 2, System Clock source is Internal RC oscillator, producing 16MHz
		SPI2Handle.SPIConfig.SlaveManagementType = SPI_SOFTWARE_SLAVE_MANAGEMENT_EN; // No actually slaves, so we can enable software slave management. 

		// Call the Init API to initialize the configured SPI2 peripheral.
		SPI_Init(&SPI2Handle);

}
int main(){
 
	// 1. Call Function to configure the I/O pins above to be used as SPI2 pin. 
	SPI2_GPIOInits();

	// 2. Call Function to Configure the SPI2 Peripheral 
	SPI2_Inits();

	// 3. Enable the SPI Peripheral

	SPI_PeripheralControl(SPI2, ENABLE); 
	
	// 4. SSI is set 1, this will pull the Master's NSS pin to High, internally. This will prevent MODF (MODE FAULT) error. 
	/* MODF error is set in the SPI_CR1_SR, to indicate that the master's NSS pin was driven to Low, 
	*  which means a different master has taken over the Bus and will generate the clock.
	*  to avoid the MODF error, when there is no slave configured in the network and Software slave management is configured, the NSS pin of the Master is internally pulled High to avoid MODF error.  
	*/
	SPI_SSIConfig(SPI2, ENABLE);

	// 5. Data to send
	char data[] ="HELLO WORLD!"; 

	// 6. Call SPI_SendData Function
	SPI_SendData(SPI2,(uint8_t *)data,strlen(data)); 

	// 7. Disable SPI Peripheral 
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);
}
