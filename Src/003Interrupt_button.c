/*
 * main.c
 *
 *  Created on: Jun 6, 2020
 *      Author: junaidkhan
 */

/**
 *          Toggle an LED when user button is pressed. The user-button is connected to PA0 and delivers an interrupt to the Processor on its Falling Edge. 
 *              The LED is toggled in the ISR.  
 * 
 */ 

#include "gpio_driver.h" 
#include <string.h>
int main(void){

    //++ 1 - Initialize LED 
    GPIO_Handle_t gpio_push_pull; 				// Instantiate a GPIO handle variable to hold pin configuration information and gain access to the API functions.

    memset(&gpio_push_pull, 0, sizeof(gpio_push_pull));  // Standard Function that sets all the registers of gpio_push_pull to zero.

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

    //-- Initialize LED 

    //++ 2 - Initialize Button on PD5 
      /*   The following statements are for configuring the user button PA0 as an input pin. */

	GPIO_Handle_t gpio_user_button; 				// Instantiate a GPIO handle variable to hold pin configuration information and gain access to the API functions.

	memset(&gpio_user_button, 0, sizeof(gpio_user_button));

	gpio_user_button.pGPIOx_BASEADDR = GPIOA; 	    // Set the base-address of the Port you're configuring - User button is connected to Port A, Pin 0.

	// Now, use the Pin structure of the GPIO handle to configure the I/O pin 0 in input mode.
	gpio_user_button.PinConfig.PinNumber = GPIO_PIN_0;
	gpio_user_button.PinConfig.PinMode = GPIO_PIN_INPUT_FALLING_EDGE_INTERRUPT_MODE; 		 // Set the pin in Falling edge interrupt mode.
	gpio_user_button.PinConfig.PinPuPdControl = GPIO_PIN_NO_PUPD;
	gpio_user_button.PinConfig.PinSpeed = GPIO_PIN_MEDIUM_SPEED;                           // Speed doesn't really matter

	// Now, enable clock access to GPIO Port D, use the RCC clock enable MACROS defined in the device specific header file or:
	// Use the clock control API defined in GPIO driver - the better option.

	GPIO_ClockControl(GPIOA, ENABLE);

	// Now, call the Init API to configure the physical address of Port D
	GPIO_Init(&gpio_user_button);  
    // -- Initialize Button PD5

    // ++ 3- IRQ Configurations - Set Priority and Enable IRQ on the NVIC. 

    GPIO_IRQ_Priority_Config(IRQ_NO_EXTI0, NVIC_PRIORITY_0);    // Don't need to set priority if there is only 1 interrupt enabled. 
    GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI0, ENABLE);            // Pin 0 delivers interrupt on EXTI0 line to the Processor via NVIC. 
    // -- IRQ Configurations - Set Priority and Enable IRQ on the NVIC. 

    while(1);

}

void EXTI0_IRQHandler(){
	//++ 4:  Write the application layer ISR function and store the function at the address assigned to IRQ number that will register the interrupt request made.

    //  Handle the Interrupt - call the Driver IRQ handling API here. 
    GPIO_IRQHandling(GPIO_PIN_0); // I/O Pin number 0 deliver's interrupts on the EXTI0 line

    // We want to toggle the LED, so we can use the Toggle API here. 
    GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_12); 
    //-- 4: Write the application layer ISR function and store the function at the address assigned to IRQ number that will register the interrupt request made.

}
