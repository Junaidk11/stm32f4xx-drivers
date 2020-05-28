/*
 * gpio_driver.h
 *
 *  Created on: May 18, 2020
 *      Author: Junaid
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stm32f40xx.h" /*  Include device specific header file. */

/*
 * 						A GPIO pin configuration structure.
 * 		 			This structure holds the configuration settings of the pin.
 * 			 Place the configuration Structure inside the handle structure.
 */

typedef struct{

	uint8_t PinNumber;				/* Pin number is b/w 0-15 - select from @GPIO_PIN_NUMBERS  */
	uint8_t PinMode;				/* select from @GPIO_MODES */
	uint8_t PinSpeed;				/* Pin's Slew rate: How fast does pin change state - select from @GPIO_PIN_SPEEDS */
	uint8_t PinPuPdControl;  		/* Pin's Internal Pull-up or Pull-down Control - select from @GPIO_PUPD_CONFIGURATIONS */
	uint8_t PinOType; 				/* Pin Output type: Open-drain configuration/Push-pull Configuration - select from @GPIO_OUTPUT_CONFIGURATIONS */
	uint8_t PinAltFunMode; 			/* Pin's alternate Functionality Mode value is placed in here.  */
}GPIO_PinConfig_t;


/*
 * 				A Handle Structure for a GPIO pin.
 * 			Holds pin's configuration and Base address of the Port being configured.
 */

typedef struct{

	GPIO_RegDef_t *pGPIOx_BASEADDR;   /* Holds the Base address of the desired Port (A-K), the Base addresses defined in device header as MACROS, GPIOA, GPIOB... */
	GPIO_PinConfig_t PinConfig; 	  /* Holds the pin configuration information - filled out by the user before calling using any of the API functions defined in this file. */

}GPIO_Handle_t;


/*
 * 					APIs supported by this GPIO driver.
 *
 */


/*
 *  		GPIO Port Clock Access
 *
 *  	This API function will be used to enable clock access to one of the GPIO Ports.
 *  	Therefore: The arguments of this function should be:
 *  		1) a pointer to the base address of the port we want to configure - GPIO_RegDef_t -> points to the base-address of the port
 *  		2) a variable to indicate if you wish to enable or disable the port. Define a Generic macro in MCU header file, use it - Use MACRO: Enable or Disable.
 *
 */

void GPIO_ClockControl(GPIO_RegDef_t *pGPIO_PORT,uint8_t enable_disable );

/*
 * 			GPIO Port Init and Deinit
 *
 * 			For Initialization of GPIO port register:
 * 		This API function will configure the pin that you wish to use.
 * 		The Arguments of this function should be:
 * 		1) Pointer to the GPIO handle; GPIO_Handle_t *
 *
 * 			For Reseting the GPIO port registers:
 * 		You can use RCC's AHB1 Peripheral Reset Register.
* 		This way, you don't have to reset each every register of the Port.
* 		The Argument of this function should be:
* 		1) The Base address of your GPIO port - GPIO_RegDef_t*
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIO_PORT);


/*
 *   		Data Read & Write
 */

 /* Arguments: To read a pin, you need GPIO Base address & pin number. The returned value will either be 0 or 1, uint8_t is the closest data type to return. */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIO_PORT, uint8_t pinNumber);

/* Arguments: To read a PORT, you need GPIO Base address. The returned value will be 16 bits long, each PORT has 16 pins. uint16_t is the closest data type to return. */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIO_PORT);

/* Arguments: To write to a pin, you need GPIO Base address, pin number & the value (use Generic Macros defined in MCU header file.) you want to write to the pin.*/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIO_PORT, uint8_t pinNumber, uint8_t value);

/* Arguments: To write to a PORT, you need GPIO Base address & the value you want to write to the  port- value has to 16 bits long, each Port has 16 pins. */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIO_PORT, uint16_t Value);

/* Arguments: To toggle a specific output pin, you only need the Base address of the GPIO port & the pin number you wish to toggle. */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO_PORT, uint8_t pinNumber);



/*
 *			GPIO Interrupt Configuration & Handling
 */

/* Arguments: You need IRQ number, the interrupt priority, and variable to hold enable or disable command. */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enable_disable);

/*  For Interrupt handling, this API only needs to know the pin number that needs interrupt servicing. */
void GPIO_IRQHandling(uint8_t pinNumber);




/*
 *     GPIO driver specific MACROs  - these macros are only relevant to GPIO driver, which is why we define them in the GPIO driver's header file & not in the MCU specific header file.
 *
 */


/*
 * GPIO possible pins - @GPIO_PIN_NUMBERS
 *
 */
#define GPIO_PIN_0 					0
#define GPIO_PIN_1				    1
#define GPIO_PIN_2					2
#define GPIO_PIN_3					3
#define GPIO_PIN_4					4
#define GPIO_PIN_5					5
#define GPIO_PIN_6					6
#define GPIO_PIN_7					7
#define GPIO_PIN_8					8
#define GPIO_PIN_9					9
#define GPIO_PIN_10					10
#define GPIO_PIN_11					11
#define GPIO_PIN_12					12
#define GPIO_PIN_13					13
#define GPIO_PIN_14					14
#define GPIO_PIN_15					15


/*
 *
 * GPIO pin possible Modes - from Reference Manual of your MCU, under GPIO Registers  @GPIO_MODES
 *
 */

/* The following are uninterrupted modes */
#define GPIO_PIN_INPUT_MODE  							0
#define GPIO_PIN_OUTPUT_MODE  							1
#define GPIO_PIN_ALTFUNC_MODE  							2
#define GPIO_PIN_ANALOG_MODE  							3

/* The following are interrupt modes of the API, it needs the EXTI module to be configured. EXTI = External Interrupt Peripheral*/
#define GPIO_PIN_INPUT_FALLING_EDGE_INTERRUPT_MODE 		4
#define GPIO_PIN_INPUT_RISING_EDGE_INTERRUPT_MODE		5
#define GPIO_PIN_INPUT_RISING_FALLING_INTERRUPT_MODE	6

/*
 *
 * GPIO output pin possible configuration - from Reference Manual of your MCU, under GPIO Registers- @GPIO_OUTPUT_CONFIGURATIONS
 *
 *  */

#define GPIO_PUSH_PULL_OUTPUT_CONFIG 					0
#define GPIO_OPEN_DRAIN_OUTPUT_CONFIG					1

/*
 *  GPIO Output pin slew rate settings - @GPIO_PIN_SPEEDS
 *
 */

#define GPIO_PIN_LOW_SPEED 								0
#define GPIO_PIN_MEDIUM_SPEED							1
#define GPIO_PIN_HIGH_SPEED								2
#define GPIO_PIN_VERY_HIGH_SPEED						3


/*
 *
 * GPIO pin pull-up and pull-down configuration - @GPIO_PUPD_CONFIGURATIONS
 *
 *
 *  */

#define GPIO_PIN_NO_PUPD  								0					// PU = Pull-up, PD = Pull-down
#define GPIO_PIN_PULL_UP								1
#define GPIO_PIN_PULL_DOWN								2

/*
 *   GPIO Set & clear MACROS - @GPIO_PIN_SET_CLEAR_MACROS
 */

#define GPIO_PIN_SET 		1
#define GPIO_PIN_CLEAR		0


#endif /* INC_GPIO_DRIVER_H_ */
