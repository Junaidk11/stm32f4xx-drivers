/*
 * gpio_driver.c
 *
 *  Created on: May 18, 2020
 *      Author: Junaid
 */

#include "gpio_driver.h" /* include its header file. */


/*
 *  		GPIO Port Clock Access
 *
 *  	This API function will be used to enable clock access to one of the GPIO Ports.
 *  	Therefore: The arguments of this function should be:
 *  		1) a pointer to the base address of the port we want to configure - GPIO_RegDef_t -> points to the base-address of the port
 *  		2) a variable to indicate if you wish to enable or disable the port. Define a Generic macro in MCU header file, use it - Use MACRO: Enable or Disable.
 *
 */
/*********************************************************************
 * @fn      		  - GPIO_ClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */


void GPIO_ClockControl(GPIO_RegDef_t *pGPIO_PORT,uint8_t enable_disable ){

	/*
	 *  Check if the enable_disable arugment is enable or disable, and then use the
	 *  clock enable macros that we have written in the MCU specific file.
	 */
	if(enable_disable == ENABLE){

		if(pGPIO_PORT == GPIOA){
			GPIOA_PERIPH_CLOCK_EN(); // GPIOA enable clock access MACRO defined in MCU specific header file.
		}else if(pGPIO_PORT == GPIOB){
			GPIOB_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOC){
			GPIOC_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOD){
			GPIOD_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOE){
			GPIOE_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOF){
			GPIOF_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOG){
			GPIOG_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOI){
			GPIOI_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOJ){
			GPIOJ_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOK){
			GPIOK_PERIPH_CLOCK_EN();
		}
	}else if(enable_disable == DISABLE){
		if(pGPIO_PORT == GPIOA){
			GPIOA_PERIPH_CLOCK_DI(); // GPIOA disable clock access MACRO defined in MCU specific header file.
		}else if(pGPIO_PORT == GPIOB){
			GPIOB_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOC){
			GPIOC_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOD){
			GPIOD_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOE){
			GPIOE_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOF){
			GPIOF_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOG){
			GPIOG_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOI){
			GPIOI_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOJ){
			GPIOJ_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOK){
			GPIOK_PERIPH_CLOCK_DI();
		}
	}
}

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

/*********************************************************************
 * @fn      		  -  GPIO_Init
 *
 * @brief             - This function is used to configure the GPIO port and pin.
 *
 * @param[in]         - Holds pin's configuration and Base address of the Port being configured.
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -

*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint8_t temp = 0;   // Use this temp variable to store the value that will be assigned to the register of the specific GPIO Port

	if(pGPIOHandle->PinConfig.PinMode <= GPIO_PIN_ANALOG_MODE){ // i.e. the GPIO pin mode selected is a non-interrupt mode

				// 1. configure the mode of GPIO pin
//			The info filled by the user before calling the API function.
								// |
				temp = (pGPIOHandle->PinConfig.PinMode << (2 * pGPIOHandle->PinConfig.PinNumber)); // GPIO's MODE register has 2 bits dedicated to each pin of the PORT, hence the need for '2' for shifting to the right pin.

//			Grabs the Physical Memory address dedicated to Mode register of your desired GPIO Port
								// |
				pGPIOHandle->pGPIOx_BASEADDR->MODER = temp; 		// Assign temp value to the MODER register using the base-address of your PORT.
	}else {

		// Do this for if the selected pin mode is one of the Interrupt Modes.
	}
	// 2. Configure slew rate of the GPIO pin
	temp = 0; 			// Reset temp, can use temp for next register's value.
	temp = (pGPIOHandle->PinConfig.PinSpeed << (2 * pGPIOHandle->PinConfig.PinNumber)); // Set the value to be assigned to the Speed register.
	pGPIOHandle->pGPIOx_BASEADDR->OSPEEDR = temp; 										// Assign the value temp to the speed register of your specific GPIO port.

	// 3. Configure Pull-up/Pull-down resistor settings of the pin - to avoid floating state
	temp = 0;     		// Rest temp
	temp = (pGPIOHandle->PinConfig.PinPuPdControl << (2 * pGPIOHandle->PinConfig.PinNumber));
	pGPIOHandle->pGPIOx_BASEADDR->PUPDR = temp;

	// 4. configure the output type of the pin
	temp = 0;
	temp = (pGPIOHandle->PinConfig.PinOType << (pGPIOHandle->PinConfig.PinNumber)); // The Output type register has 1 bit dedicated for each pin of the port.
	pGPIOHandle->pGPIOx_BASEADDR->OTYPER = temp;

	// 5. configue the alternate functionality of the GPIO pin.

	if(pGPIOHandle->PinConfig.PinMode == GPIO_PIN_ALTFUNC_MODE){ // Only configure the Alternate Functionality if the user has set the Pin mode to be in Alternate Function mode, else skip this part.
				// There are two register's dedicated for selecting Alternate functionality, four bit for each pin.
				// ALFL = alternate functionalit low for pins 0-7
				// ALFH = alternate functionality high for pins 8-15
	}



}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIO_PORT){

}


/*
 *   		Data Read & Write
 */

 /* Arguments: To read a pin, you need GPIO Base address & pin number. The returned value will either be 0 or 1, uint8_t is the closest data type to return. */

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIO_PORT, uint8_t pinNumber);

/* Arguments: To read a PORT, you need GPIO Base address. The returned value will be 16 bits long, each PORT has 16 pins. uint16_t is the closest data type to return. */

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIO_PORT);

/* Arguments: To write to a pin, you need GPIO Base address, pin number & the value (use Generic Macros defined in MCU header file.) you want to write to the pin.*/

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIO_PORT, uint8_t pinNumber, uint8_t value){
}

/* Arguments: To write to a PORT, you need GPIO Base address & the value you want to write to the  port- value has to 16 bits long, each Port has 16 pins. */

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIO_PORT, uint16_t Value){

}

/* Arguments: To toggle a specific output pin, you only need the Base address of the GPIO port & the pin number you wish to toggle. */

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO_PORT, uint8_t pinNumber){

}


/*
 *			GPIO Interrupt Configuration & Handling
 */

/* Arguments: You need IRQ number, the interrupt priority, and variable to hold enable or disable command. */

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enable_disable){

}

/*  For Interrupt handling, this API only needs to know the pin number that needs interrupt servicing. */

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

*/
void GPIO_IRQHandling(uint8_t pinNumber){

}
