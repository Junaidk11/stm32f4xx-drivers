/*
 * 002led_button.c
 *
 *  Created on: May 31, 2020
 *      Author: junaidkhan
 */

#include "stm32f40xx.h"
#include "gpio_driver.h"

#define HIGH 		ENABLE
#define PRESSED 	HIGH
/*
 * 		This is the main file used for toggling an led when a button is pressed.
 *
 * 		LED = PD12 - output in push-pull configuration
 * 		Button = PA0 - input
 * 		No need for pull-up or pull down - because the user button of STM32F4-Discovery has an external pull-down resistor - check schematic.
 *
 */
void Delay(){
	int i = 0;
	for (;i<500000/2;i++);
}


int main(void){

	// The following statements use the GPIO handle to configure a Port D pin in output mode with Push-pull configuration.
	// The same steps are used to configure a pin in output mode with open-drain configuration, the only difference is
	//  - in Open drain configuration, the output can be actively driven low, but needs a pull-up resistor to drive the pin high.
	// The pull-up resistor can be internal or external. Use internal unless you need to control the amount of current sourced from the output pin.

	GPIO_Handle_t gpio_push_pull; 				// Instantiate a GPIO handle variable to hold pin configuration information and gain access to the API functions.
	gpio_push_pull.pGPIOx_BASEADDR = GPIOD; 	// Set the baseaddress of the Port you're configuring - Green LED is connected to Port D, Pin 12.

	// Now, use the Pin structure of the GPIO handle to configure the I/O pin 12 in Push-pull configuration.
	gpio_push_pull.PinConfig.PinNumber = GPIO_PIN_12;
	gpio_push_pull.PinConfig.PinMode = GPIO_PIN_OUTPUT_MODE; 		 // Set the pin in Output mode.
	gpio_push_pull.PinConfig.PinOType = GPIO_PUSH_PULL_OUTPUT_CONFIG; //Set the I/O pin in PUSH-PULL configuration of Output mode
	gpio_push_pull.PinConfig.PinPuPdControl = GPIO_PIN_NO_PUPD;      // In Push-pull configuration, don't need pull-up or pull-down resistor.
	gpio_push_pull.PinConfig.PinSpeed = GPIO_PIN_MEDIUM_SPEED;   // Speed doesn't really matter

	// Now, enable clock access to GPIO Port D, use the RCC clock enable MACROS defined in the device specific header file or:
	// Use the clock control API defined in GPIO driver - the better option.

	GPIO_ClockControl(GPIOD, ENABLE);

	// Now, call the Init API to configure the physical address of Port D
	GPIO_Init(&gpio_push_pull);


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

	while(1){

		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)== PRESSED){

			Delay(); 								  // Set a software delay - to prevent pin from de-bouncing.
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12); // Toggle Pin 12
		}
	}
   return 0;


}


