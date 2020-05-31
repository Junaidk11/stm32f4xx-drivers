/*
 * 001led_toggle.c
 *
 *  Created on: May 27, 2020
 *      Author: Junaid
 */

/*
 *  This main file is created to test the GPIO driver without interrupts.
 *
 *  Task: Toggle the green LED of the STM32 Discovery board. The respective GPIO pin (Port D, Pin 12) will be
 *  configured first in Push-pull configuration, then in Open-drain configuration.
 *
 */

#include "stm32f40xx.h" // Include device specific header file, to get access to the inner layout of your target.
#include "gpio_driver.h" // Include GPIO driver header file, this will give you access to APIs that can be used to initialize and use the GPIO peripheral of your target.

void Delay(){
	int i = 0;
	for (;i<5000;i++);
}
int main(void){

	// The following statements use the GPIO handle to configure a Port D pin in output mode with Push-pull configuration.
	// The same steps are used to configure a pin in output mode with open-drain configuration, the only difference is
	//  - in Open drain configuration, the output can be actively driven low, but needs a pull-up resistor to drive the pin high.
	// The pull-up resistor can be internal or external. Use internal unless you need to control the amount of current sourced from the output pin.

	GPIO_Handle_t gpio_push_pull; 				// Instantiate a GPIO handle variable to hold pin configuration information and gain access to the API functions.
	gpio_push_pull.pGPIOx_BASEADDR = GPIOD; 	// Set the baseaddress of the Port you're configuring - Green LED is connected to Port D, Pin 12.

	// Now, use the Pin structure of the GPIO handle to configure the I/O pin 12 in Push-pull configuration.
	gpio_push_pull.PinConfig.PinNumber = GPIO_PIN_14;
	gpio_push_pull.PinConfig.PinMode = GPIO_PIN_OUTPUT_MODE; 		 // Set the pin in Output mode.
	gpio_push_pull.PinConfig.PinOType = GPIO_PUSH_PULL_OUTPUT_CONFIG; //Set the I/O pin in PUSH-PULL configuration of Output mode
	gpio_push_pull.PinConfig.PinPuPdControl = GPIO_PIN_NO_PUPD;      // In Push-pull configuration, don't need pull-up or pull-down resistor.
	gpio_push_pull.PinConfig.PinSpeed = GPIO_PIN_MEDIUM_SPEED;   // Speed doesn't really matter

	// Now, enable clock access to GPIO Port D, use the RCC clock enable MACROS defined in the device specific header file or:
	// Use the clock control API defined in GPIO driver - the better option.

	GPIO_ClockControl(GPIOD, ENABLE);

	// Now, call the Init API to configure the physical address of Port D
	GPIO_Init(&gpio_push_pull);


	while(1){

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_14); // Toggle Pin 12
		Delay(); 								  // Set a software delay
	}

   return 0;


}

