/*
 * 001led_toggle.c
 *
 *  Created on: July 11, 2020
 *      Author: Junaid
 */

#include "stm32f40xx.h"
#include "gpio_driver.h"
#include "spi_driver.h"



/**
 *  Command Codes
 */ 

#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

/**
 *  LED On/Off Macros
 */
#define LED_ON                  1
#define LED_OFF                 0

/**
 *  Arduino Analog Pins
 */
#define ANALOG_PIN0             0
#define ANALOF_PIN1             1
#define ANALOF_PIN2             2
#define ANALOF_PIN3             3
#define ANALOF_PIN4             4

/**
 *  LED Pin connected to Pin 9 of Arduino
 */ 
#define LED_PIN     9

/**
 *  ACK Code definition
 */

#define ACK_CODE  0xF5

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
	SPIPins.PinConfig.PinPuPdControl = GPIO_PIN_PULL_UP;  // Don't need PU/PD for Push-pull output configuration
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

uint8_t SPI_VerifyResponse(uint8_t ackByte){

    if(ackByte == ACK_CODE){
        return 1;
    }
    return 0;
}
/**
 *          What is happening in the main function?
 *        1. Initialize SPI2 peripheral first. (Need to set AF of GPIO pins to be used as SPI2)
 * `    ` 2. Wait till the Button is pressed. 
 *        3. When button is pressed, send Command to Arduino, followed by dummy read to clear RXNE flag. 
 *        4. Send dummy byte to Slave, and read the ACK/NACK response from Slave.
 *        5. If ACK received, send argumnents of the command, if NACK received, diplay error on display. 
 *        6. Wait till Button pressed again. 
 *        7. Repeat steps 3-5. 
 * 
 */ 

int main(){

    //              ++ INITIALIZE THE PERIPHERALS 

            // 1. Call Function to configure the I/O pin PA0 as button 
            GPIO_ButtonInits();

            // 2. Call Function to configure the I/O pins above to be used as SPI2 pin. 
            SPI2_GPIOInits();

            // 3. Call Function to Configure the SPI2 Peripheral 
            SPI2_Inits();

            // 4. clear SSOE pin to Disable Software Slave Managment, as ST is connected to Arduino Slave
            SPI_SSOEConfig(SPI2, DISABLE);

    //              -- INITIALIZE THE PERIPHERALS 

    uint8_t dummy_read; 
    uint8_t dummy_write = 0xFF; 
    uint8_t slave_response; 
    uint8_t arguments[2]; 


    while(1){

        // Wait till button is pressed 
        while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)); 

        // To avoid Button de-bouncing, add software delay
        Delay();

        // Enable SPI2
        SPI_PeripheralControl(SPI2, ENABLE); 

        // Send LED Control Command 
        // CMD_LED_CTRL  <Pin number> <Value>
        uint8_t command = COMMAND_LED_CTRL; 
        
        // Send LED command first
        SPI_SendData(SPI2, &command, 1);

        // Dummy read, to clear RXNE flag
        SPI_ReceiveData(SPI2, &dummy_read, 1);

        //Send Dummy byte, to read slave response for the command that was send earlier. 
        //Master always initiates communication, so send dummy bytes to get slave response. 
        SPI_SendData(SPI2, &dummy_write, 1); 

        // Read Slave response
        SPI_ReceiveData(SPI2, &slave_response, 1); 

       // Check if slave responsed with ACK or NACK 

       if(SPI_VerifyResponse(slave_response)){

           // Recieved acknowledment from Slave, arguments of the command you sent befoe
            arguments[0] = LED_PIN; // Argument 1 is Pin number
            arguments[1] = LED_ON;  //  Turn LED on

           //  Send Arguments to Slave
           SPI_SendData(SPI2, arguments, 1); // '2' because we're sending 2 arguments of 8 bits each. 
       
            // Dummy read to clear RXNE flag 

            SPI_ReceiveData(SPI2, &dummy_read, 1); 
       }

        // Confirm SPI2 is not busy 
        while(!(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)));  // While SPI is busy, you wait. 

        // SPI2 not busy anymore, disable SPI2 
        SPI_PeripheralControl(SPI2, DISABLE); 

    }


}
